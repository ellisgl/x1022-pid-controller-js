var util                = require('util');
var EventEmitter        = require('events').EventEmitter;
var extend              = require('node.extend');

/**
 * Based on:
 *   http://www.regnumelectronic.com/Docs/PID.pdf (Standard Independent Type C)
 */
function PIDController(options)
{
    // Options configuration
    this.opts       = {
        Kp         : 1.0,
        Ki         : 0.0,
        Kd         : 0.0,
        round      : true,
        Ts         : 1000,  // In MS. (Old sampleTime)
        uMin       : 0,
        uMax       : 255
    };

    options         = options || {};
    this.opts       = extend({}, this.opts, options);

    // Internal Use variables
    this.input           = function ()
    {
        // Function (Reading from sensor)
        this.setIVal(0);
    };

    this.direction       = 0;     // 0 = forward, 1 = drop down to and go forward, 3 = reverse
    this.duration        = 1;     // In seconds (How long to run - 0 = infinite)

    this.output          = function (val)
    {
        // Function (Writing to device)
        return val;
    };

    this.e          = [0.00, 0.00];
    this.PV         = [0.00, 0.00, 0.00];
    this.CO         = [0.00, 0.00];
    this.interval   = 0;
    this.iVal       = 0;
    this.running    = false;
    this.startTime  = 0;
    this.duration   = 0;
    this.SP         = 0;
    this.t1         = 0;
    this.timeLeft   = 0;
    this.tReached   = false;

    EventEmitter.call(this);
}

util.inherits(PIDController, EventEmitter);

PIDController.prototype.compute    = function()
{
    /**
     * Using Type C PID
     * k     = sample number
     * Ts    = sample period (seconds)
     * e     = error term = SP - PV(k) or PV(k) - SP
     * SP    = set point
     * PV    = input value / proccess variable
     * CO    = output value
     * Kp    = Gain of proportional control
     * Ki    = Gain of the integral control
     * Kd    = Gain of the derivative control
     * CO(k) = CO(k-1) + Ki * e(k) * Ts - Kp * (PV(k) - PV(k-1)) - Kd / Ts * (PV(k)  - 2*PV(k-1) + PV(k-2))
     *
     * a = CO(k-1) + Ki * e(k) * Ts
     * b = Kp * (PV(k) - PV(k-1))
     * c = Kd / Ts * (PV(k)  - 2*PV(k-1) + PV(k-2))
     * d = a - b - c
     */
    if(!this.running)
    {
        console.log('PID NOT RUNNING, CAN NOT COMPUTE');
        return false;
    }

    var now      = new Date().getTime();
    var tChanged = (now - this.t1);
    //var ek       = 0.0;
    this.opts.Ts = (this.opts.Ts > 0) ? this.opts.Ts : 1000; // Crazy loop preventor.


    // Make sure we don't get a wild loop messing up our plans.
    if(tChanged >= this.opts.Ts)
    {
        this.t1 = now;

        // Get current input reading and put at beggining of array and knock off last element.
        this.input();
        this.PV.unshift(this.iVal);
        this.PV.pop();

        // Get error
        this.e.unshift(parseFloat((this.direction < 3) ? this.SP - this.PV[0] : this.PV[0] - this.SP));
        this.e.pop();

        var a = this.CO[1] + this.opts.Ki * this.e[0] * this.opts.Ts;
        var b = this.opts.Kp * (this.PV[0] - this.PV[1]);
        var c = this.opts.Kd / this.opts.Ts * (this.PV[0] - (2 * this.PV[1]) + this.PV[2]);
        var d = Math.round(a - b - c);

        if(d < this.opts.uMin)
        {
            d = this.opts.uMin;
        }
        else if(d > this.opts.uMax)
        {
            d = this.opts.uMax;
        }

        this.CO.unshift(d);
        this.CO.pop();

        // Did we hit the target yet?
        // Might have to review this one!
        if((this.direction == 0 && this.tReached == false && this.PV[0] >= this.SP) ||
            (this.direction >  0 && this.tReached == false && this.PV[0] <= this.SP))
        {
            this.tReached = true;
            this.emit('targetReached');
        }
        else if(this.direction > 0 && this.tReached == false && this.PV[0] <= this.SP)
        {
            this.tReached = true;
            this.emit('targetReached');
        }

        /**
         console.log('---------------------');
         console.log(this.PV);
         console.log(this.e);
         console.log(this.CO);
         console.log(a);
         console.log(b);
         console.log(c);
         console.log(d);
         console.log('---------------------');
         **/
        return this.CO[0];
    }

    return false;
};

PIDController.prototype.getStatus       = function ()
{
    return {'running' : this.running, 'target' : this.SP, 'TargetReached': this.tReached, 'direction' : this.direction, 'duration' : this.duration, 'timeLeft' : this.timeLeft};
};

PIDController.prototype.setDirection    = function (direction)
{
    this.direction = direction;
};

PIDController.prototype.setDuration     = function (duration)
{
    this.duration = duration;
};

PIDController.prototype.setInput        = function (input)
{
    this.input = input;
};

//Ugh..
PIDController.prototype.setIVal         = function (val)
{
    this.iVal = val;
};

PIDController.prototype.setOutput       = function (output)
{
    this.output = output;
};

PIDController.prototype.setOutputLimits = function (min, max)
{
    if (min >= max)
    {
        return false; // Should throw error.
    }

    this.opts.outMin = min;
    this.opts.outMax = max;
};

PIDController.prototype.setTarget       = function (target)
{
    this.tReached = false;
    this.SP  = parseFloat(target);
};

PIDController.prototype.setTs           = function (ts)
{
    this.opts.Ts = ts;
};

PIDController.prototype.setTunings      = function (kc, ti, td)
{
    if(kc < 0 || ti < 0 || td < 0)
    {
        // Should throw err here.
        return false;
    }

    this.opts.kc = kc;
    this.opts.ti = ti;
    this.opts.td = td;
};

PIDController.prototype.start      = function ()
{
    // Make sure we can run run()
    this.running   = true;

    // Run-a-way intervals (timeouts)?
    clearTimeout(this.interval);

    // Safeties.
    this.e         = [0.00, 0.00];
    this.PV        = [0.00, 0.00, 0.00];
    this.CO        = [0.00, 0.00];
    this.interval  = 0;
    this.startTime = 0;
    this.t1        = 0;
    this.timeLeft  = 0;
    this.tReached  = false;

    console.log('PID STARTED');
    this.emit('started');
    this.run();
};

PIDController.prototype.stop       = function ()
{
    // Make sure run() can't be triggered.
    this.running  = false;

    // Set the output to 0
    this.output(0);

    // Stop the interval (timeout)
    clearTimeout(this.interval);

    // Set back to defaults.
    this.e          = [0.00, 0.00];
    this.PV         = [0.00, 0.00, 0.00];
    this.CO         = [0.00, 0.00];
    this.interval   = 0;
    this.iVal       = 0;
    this.running    = false;
    this.startTime  = 0;
    this.duration   = 0;
    this.SP         = 0;
    this.t1         = 0;
    this.timeLeft   = 0;

    console.log('PID STOPPED');
    this.emit('stopped');
};

PIDController.prototype.run        = function ()
{
    if(!this.running)
    {
        console.log('PID NOT RUNNING, CAN NOT RUN');
        return false;
    }

    var self      = this;
    var now       = new Date().getTime() / 1000;
    var out       = this.compute();

    if(this.startTime == 0 && this.tReached == true)
    {
        this.startTime = now;
    }

    //console.log('NOW:', now, 'START TIME:', this.startTime);

    // Only change the output if there is a change.
    // Love mixing things up.
    if(out != false && this.CO[0] != this.CO[1])
    {
        this.output(Math.round(out));
    }

    if(this.startTime > 0 && this.tReached == true)
    {
        this.timeLeft = this.duration - (now - this.startTime);
        //console.log('TIME LEFT', this.timeLeft)

        if (this.timeLeft <= 0.0)
        {
            console.log('STEP DURATION HIT');
            this.stop();
        }
    }

    this.interval = setTimeout(function ()
    {
        self.run();
    }, this.opts.Ts);
};

module.exports         = PIDController;