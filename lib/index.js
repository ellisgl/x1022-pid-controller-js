var extend        = require('node.extend');

var PIDController = function(options)
{
    // Options configuration
    var defaults     = {
        // Function (Reading from sensor)
        input      : function()
        {
            return 0;
        },
        // Function (Writing to device)
        output     : function()
        {
            return 0;
        },
        target     : 0,
        sampleTime : 250,
        Kp         : 1.0,
        Ki         : 0.0,
        Kd         : 0.0,
        min        : 0,
        max        : 255,
        round      : true,
        direction  : 0    // 0 = forward, 1 = reverse
    };

    options         = options || {};
    this.opts       = extend({}, defaults, options);
    this.iVal       = 0;
    this.iValLast   = 0;
    this.oVal       = 0;
    this.oValLast   = 0;
    this.lTime      = 0;
    this.kp         = 0.0;
    this.ki         = 0.0;
    this.kd         = 0.0;
    this.dispKp     = 0.0;
    this.dispKi     = 0.0;
    this.dispKd     = 0.0;
    this.running    = false;
    this.tReached   = false;
    this.iTerm      = 0;
    this.interval   = 0;

    this.setDirection(this.opts.direction);
    this.setTunings(this.opts.Kp, this.opts.Ki, this.opts.Kd);
};

PIDController.prototype = {
    compute         : function ()
    {
        if(!this.running)
        {
            return false;
        }

        var d        = new Date();
        var now      = d.getTime();
        var tChanged = (now - this.lTime);

        if(tChanged >= this.opts.sampleTime)
        {
            /*Compute all the working error variables*/
            this.opts.input();

            var input      = this.iVal;
            var error      = parseFloat(this.opts.target) - input;
            this.iTerm    += (this.ki * error);

            if(this.iTerm > this.opts.max)
            {
                this.iTerm = this.opts.max;
            }
            else if(this.iTerm < this.opts.min)
            {
                this.iTerm= this.opts.min;
            }

            var dInput     = parseFloat(input - this.iValLast);

            /*Compute PID Output*/
            var output = (this.kp * error + this.iTerm - this.kd * dInput);

            if(output > this.opts.max)
            {
                output = this.opts.max;
            }
            else if(output < this.opts.min)
            {
                output = this.opts.min;
            }

            output         = (this.opts.round) ? Math.round(output) : output;

            if(output != this.oValLast)
            {
                this.oValLast  = output;
                this.opts.output(output);
            }

            /*Remember some variables for next time*/
            this.iValLast  = input;
            this.lTime     = now;

            /** Did we hit the target yet? **/
            //console.log('TARGET: ', this.opts.target);
            //console.log('INPUT: ', input);

            if(this.tReached == false && input >= this.opts.target)
            {
                this.tReached = true;
                //console.log('TARGET REACHED!');
            }

            return true;
        }

        // Not time yet.
        return false;
    },
    run            : function()
    {
        if (!this.running)
        {
            return false;
        }

        this.compute();

        var self      = this;
        this.interval = setTimeout(function(){ self.run(); }, this.opts.sampleTime);

        return true;
    },
    setDirection    : function (direction)
    {
        if(direction != this.opts.direction)
        {
            // Switch to and from a negative number
            this.kp = (0 - this.kp);
            this.ki = (0 - this.ki);
            this.kd = (0 - this.kd);
        }

        this.opts.direction = direction;
        return true;
    },
    setOutputLimits : function (min, max)
    {
        if(min >= max)
        {
            return false; // Should throw error.
        }

        // Probably shouldn't be doing this with the OPTS.
        this.opts.min = min;
        this.opts.max = max;

        if(this.running)
        {
            if(this.oVal > this.opts.max)
            {
                this.oVal = this.opts.max;
            }
            else if(this.oVal < this.opts.min)
            {

                this.oVal = this.opts.min;
            }

            if(this.iTerm > this.opts.max)
            {
                this.iTerm = this.opts.max;
            }
            else if(iTerm < this.opts.min)
            {
                this.iTerm = this.opts.min;
            }
        }

        return true;
    },
    setSampleTime   : function (newSampleTime)
    {
        if(newSampleTime > 0)
        {
            var ratio             = parseFloat(newSampleTime / this.opts.sampleTime);
            this.ki              *= ratio;
            this.kd              /= ratio;
            this.opts.sampleTime  = NewSampleTime;

            return true;
        }

        return false;
    },
    setInput        : function (input)
    {
        this.opts.input = input;
        return true;
    },
    setOutput        : function (output)
    {
        this.opts.output = output;
        return true;
    },
    setTarget       : function (target)
    {
        this.tReached    = false;
        this.opts.target = parseFloat(target);
        return true;
    },
    setTunings      : function (Kp, Ki, Kd)
    {
        if(Kp < 0 || Ki < 0 || Kd < 0)
        {
            // Should throw err here.
            return false;
        }

        this.dispKp         = Kp;
        this.dispKi         = Ki;
        this.dispKd         = Kd;
        var sampleTimeInSec = parseFloat(this.opts.sampleTime/1000);

        this.kp = Kp;
        this.ki = Ki * sampleTimeInSec;
        this.kd = Kd / sampleTimeInSec;

        if(this.opts.direction == 1)
        {
            this.kp = (0 - this.kp);
            this.ki = (0 - this.ki);
            this.kd = (0 - this.kd);
        }

        return true;
    },
    start           : function()
    {
        console.log('starting');
        this.iTerm    = this.oVal;
        this.iValLast = this.iVal;
        this.running  = true;

        // Run-a-way intervals (timeouts)?
        clearTimeout(this.interval);

        if(this.iTerm > this.opts.max)
        {
            this.iTerm = this.opts.max;
        }
        else if(this.iTerm < this.opts.min)
        {
            this.iTerm = this.opts.min;
        }

        this.run();
        return true;
    },
    setIVal        : function(val)
    {
        this.iVal = val;
        return true;
    },
    status         : function()
    {
        return {'running' : this.running, 'target' : this.opts.target, 'tReached': this.tReached, 'direction' : this.opts.direction};
    },
    stop            : function()
    {
        this.running  = false;
        clearTimeout(this.interval);
        return true;
    }
}

module.exports    = PIDController;