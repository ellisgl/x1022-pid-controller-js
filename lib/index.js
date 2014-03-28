var util                = require('util');
var EventEmitter        = require('events').EventEmitter;
var extend              = require('node.extend');

/**
 * Based on:
 *   https://github.com/steve71/RasPiBrew
 *   http://www.embedded.com/design/configurable-systems/4212241/Case-Study-of-PID-Control-in-an-FPGA-
 *   http://www.vandelogt.nl/htm/regelen_pid_uk.htm
 */
function PIDController(options)
{
    // Options configuration
    this.opts       = {
        KP         : 1.0,   // Old Kp
        KI         : 0.0,   // Old Ki
        KD         : 0.0,   // Old Kd
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
        return 0;
    };

    this.direction       = 0;     // 0 = forward, 1 = drop down to and go forward, 3 = reverse
    this.duration        = 1;     // In seconds (How long to run - 0 = infinite)

    this.output          = function (val)
    {
        // Function (Writing to device)
        return val;
    };

    this.yk         = [0.00, 0.00, 0.00];
    this.uk         = [0.00, 0.00];
    this.interval   = 0;
    this.iVal       = 0;
    this.running    = false;
    this.startTime  = 0;
    this.SP         = 0;     // Set point (target)
    this.t1         = 0;
    this.timeLeft   = 0;
    this.tReached   = false;

    EventEmitter.call(this);
}

util.inherits(PIDController, EventEmitter);

PIDController.prototype.compute    = function()
{
    // Using calcPID_reg4 (Type C PID)
    if(!this.running)
    {
        console.log('PID NOT RUNNING, CAN NOT COMPUTE');
        return false;
    }

    var now      = new Date().getTime();
    var tChanged = (now - this.t1);
    var ek       = 0.0;
    var U        = 0.0;
    var U0       = 0.0;
    var U1       = 0.0;
    var U2       = 0.0;
    this.opts.Ts = (this.opts.Ts > 0) ? this.opts.Ts : 1000; // Crazy loop preventor.


    // Make sure we don't get a wild loop messing up our plans.
    if(tChanged >= this.opts.Ts)
    {
        this.t1 = now;
        var Tss = this.opts.Ts / 1000; // Convert Ts into seconds
        /**
         * u(k) = u(k - 1) + Kp(y(k - 1) - y(k)) + Ki * Ts * e(k) + Kd / Ts * (2y(k - 1) - y(k) - y(k - 2))
         *
         * k     = sample number
         * Ts    = sample period (seconds)
         * e(k)  = error term = SP(K) - y(k)
         * SP(k) = set point
         * y(k)  = input value
         * u(k)  = output value
         * KP    = Gain of proportional control
         * KI    = Gain of the integral control
         * KD    = Gain of the derivative control
         */
            // Get current input reading and put at beggining of array and knock off last element.
        this.input();
        this.yk.pop();
        this.yk.unshift(this.iVal);
        console.log(this.yk);

        // Get error
        ek      = parseFloat((this.direction < 3) ? this.SP - this.yk[0] : this.yk[0] - this.SP);
        U0      = this.uk[1] + this.opts.KP * (this.yk[1] - this.yk[0]);                 // u(k - 1) + Kp(y(k - 1) - y(k))
        U1      = this.opts.KI * Tss * ek;                                               // Ki * Ts * e(k)
        U2      = (this.opts.KD / Tss)  * ((2 * this.yk[1]) - this.yk[0] - this.yk[2]);  // Kd / Ts * (2y(k - 1) - y(k) - y(k - 2)
        U       = U0 + U1 + U2;

        console.log('------------');
        console.log('SP', this.SP);
        console.log('yk', this.yk);
        console.log('uk', this.uk);
        console.log('KP', this.opts.KP);
        console.log('KI', this.opts.KI);
        console.log('KD', this.opts.KD);
        console.log('Ts', this.opts.Ts);
        console.log('Tss', Tss);
        console.log('ek', ek);
        console.log('U0', U0);
        console.log('U1', U1);
        console.log('U2', U2);
        console.log('U', U);
        console.log('------------');

        if(U < this.opts.uMin)
        {
            U = this.opts.uMin;
        }
        else if(U > this.opts.uMax)
        {
            U = this.opts.uMax;
        }

        this.uk.pop();
        this.uk.unshift(U);

        // Did we hit the target yet?
        // Might have to review this one!
        if((this.direction == 0 && this.tReached == false && this.yk[0] >= this.SP) ||
            (this.direction >  0 && this.tReached == false && this.yk[0] <= this.SP))
        {
            this.tReached = true;
            this.emit('targetReached');
        }
        else if(this.direction > 0 && this.tReached == false && this.yk[0] <= this.SP)
        {
            this.tReached = true;
            this.emit('targetReached');
        }

        return this.uk[0];
    }

    return false;
};

PIDController.prototype.getStatus       = function ()
{
    return {'running' : this.running, 'target' : this.SP, 'TargetReached': this.tReached, 'direction' : this.direction, 'duration' : this.duration, 'timeLeft' : this.timeLeft};
};

PIDController.prototype.setDirection    = function (direction)
{
    this.directions = direction;
    return this;
};

PIDController.prototype.setDuration     = function (duration)
{
    this.duration = duration;
    return this;
};

PIDController.prototype.setInput        = function (input)
{
    this.input = input;
    return this;
};

//Ugh..
PIDController.prototype.setIVal         = function (val)
{
    this.iVal = val;
    return this;
};

PIDController.prototype.setOutput       = function (output)
{
    this.output = output;
    return this;
};

PIDController.prototype.setOutputLimits = function (min, max)
{
    if (min >= max)
    {
        return false; // Should throw error.
    }

    this.opts.outMin = min;
    this.opts.outMax = max;

    return this;
};

PIDController.prototype.setTarget       = function (target)
{
    this.tReached = false;
    this.SP  = parseFloat(target);

    return this;
};

PIDController.prototype.setTs           = function (ts)
{
    this.opts.Ts = ts;
    return this;
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

    return this;

};

PIDController.prototype.start      = function ()
{
    // Make sure we can run run()
    this.running   = true;

    // Run-a-way intervals (timeouts)?
    clearTimeout(this.interval);

    // Safeties.
    this.interval  = 0;
    this.yk        = [0.00, 0.00, 0.00];
    this.uk        = [0.00, 0.00];
    this.startTime = 0;
    this.t1        = 0;
    this.timeLeft  = 0;
    this.tReached  = false;

    this.emit('started');
    this.run();
    return this;
};

PIDController.prototype.stop       = function ()
{
    // Make sure run() can't be triggered.
    this.running  = false;

    // Stop the interval (timeout)
    clearTimeout(this.interval);

    // Set the output to 0
    this.output(0);

    this.emit('stopped');
    return this;
}
PIDController.prototype.run        = function ()
{

    if (!this.running)
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

    // Only change the output if there is a change.
    // Love mixing things up.
    if(out != false && this.uk[0] != this.uk[1])
    {
        this.output(out);
    }

    this.timeLeft = now - this.startTime;

    if(this.timeLeft > this.duration)
    {
        this.interval = setTimeout(function ()
        {
            self.run();
        }, this.opts.Ts);
    }
    else
    {
        this.stop();
    }

    return this;
};

module.exports         = PIDController;