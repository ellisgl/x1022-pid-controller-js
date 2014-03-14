var util          = require('util');
var events        = require('events');
var extend        = require('node.extend');

var PIDController = function(options)
{
    // Options configuration
    this.opts     = {
        // Function (Reading from sensor)
        input      : function()
        {
            return 0;
        },
        // Function (Writing to device)
        output     : function(val)
        {
            return val;
        },
        target     : 0,
        sampleTime : 1000,  // In MS.
        Kp         : 1.0,
        Ki         : 0.0,
        Kd         : 0.0,
        outMin     : 0,
        outMax     : 255,
        round      : true,
        duration   : 1,    // In seconds (How long to run - 0 = infinite)
        direction  : 0     // 0 = forward, 1 = reverse
    };

    options         = options || {};
    this.opts       = extend({}, this.opts, options);
    this.iVal       = 0;
    this.iValLast   = 0;
    this.oVal       = 0;
    this.oValLast   = 0;
    this.lTime      = 0;
    this.kp         = 0.0;
    this.ki         = 0.0;
    this.kd         = 0.0;
    this.running    = false;
    this.tReached   = false;
    this.iTerm      = 0;
    this.interval   = 0;
    this.startTime  = 0;

    this.setDirection(this.opts.direction);
    this.setTunings(this.opts.Kp, this.opts.Ki, this.opts.Kd);

    // Setup the Events
    events.EventEmitter.call(this);
};

PIDController.prototype = {
    compute         : function ()
    {
        if(!this.running)
        {
            return false;
        }

        var d                = new Date();
        var now              = d.getTime();
        var tChanged         = (now - this.lTime);

        // Make sure we don't get any run away timeouts.
        this.opts.sampleTime = (this.opts.sampleTime > 0) ? this.opts.sampleTime : 1000;

        if(tChanged >= this.opts.sampleTime)
        {
            /*Compute all the working error variables*/
            this.opts.input();

            var input      = this.iVal;
            var error      = parseFloat(this.opts.target) - input;
            this.iTerm    += (this.ki * error);

            if(this.iTerm > this.opts.outMax)
            {
                this.iTerm = this.opts.outMax;
            }
            else if(this.iTerm < this.opts.outMin)
            {
                this.iTerm= this.opts.outMin;
            }

            var dInput     = parseFloat(input - this.iValLast);

            /*Compute PID Output*/
            var output = (this.kp * error + this.iTerm - this.kd * dInput);

            if(output > this.opts.outMax)
            {
                output = this.opts.outMax;
            }
            else if(output < this.opts.outMin)
            {
                output = this.opts.outMin;
            }

            output         = (this.opts.round) ? Math.round(output) : output;

            if(output != this.oValLast)
            {
                this.oValLast  = output;
                this.opts.output(output);
            }

            // Remember some values for next time
            this.iValLast  = input;
            this.lTime     = now;

            // Did we hit the target yet?
            if(this.opts.direction == 0 && this.tReached == false && input >= this.opts.target)
            {
                this.tReached = true;
                this.emit('targetReached');
            }
            else if(this.opts.direction == 1 && this.tReached == false && input <= this.opts.target)
            {
                this.tReached = true;
                this.emit('targetReached');
            }

            return this;
        }

        // Not time yet.
        return false;
    },
    run            : function()
    {
        var self      = this;
        var d         = new Date();
        var now       = d.getTime() / 1000;

        if(this.startTime == 0)
        {
            this.startTime = d.getTime() / 1000;
        }

        if (!this.running)
        {
            return false;
        }

        this.compute();

        this.timeLeft = now - this.startTime;

        if(this.timeLeft > this.opts.duration)
        {
            this.interval = setTimeout(function ()
            {
                self.run();
            }, this.opts.sampleTime);
        }
        else
        {
            this.stop();
        }

        return this;
    },
    setDirection    : function (direction)
    {
        if(direction == 0)
        {
            // Positive (upward) gain
            if(this.kp < 0)
            {
                // Make sure it's a negative number before changing to positive.
                this.kp = (0 - this.kp);
                this.ki = (0 - this.ki);
                this.kd = (0 - this.kd);
            }
        }
        else
        {
            // Negative (downward direction)
            if(this.kp > 0)
            {
                // Make sure it's a positive number before changing to negative.
                this.kp = (0 - this.kp);
                this.ki = (0 - this.ki);
                this.kd = (0 - this.kd);
            }
        }

        this.opts.direction = direction;
        return this;
    },
    setDuration    : function (duration)
    {
        this.opts.duration = duration;
        return this;
    },
    setInput        : function (input)
    {
        this.opts.input = input;
        return this;
    },
    setIVal        : function(val)
    {
        this.iVal = val;
        return this;
    },
    setOutputLimits : function (min, max)
    {
        if(min >= max)
        {
            return false; // Should throw error.
        }

        // Probably shouldn't be doing this with the OPTS.
        this.opts.outMin = min;
        this.opts.outMax = max;

        if(this.running)
        {
            if(this.oVal > this.opts.outMax)
            {
                this.oVal = this.opts.outMax;
            }
            else if(this.oVal < this.opts.outMin)
            {

                this.oVal = this.opts.outMin;
            }

            if(this.iTerm > this.opts.outMax)
            {
                this.iTerm = this.opts.outMax;
            }
            else if(this.iTerm < this.opts.outMin)
            {
                this.iTerm = this.opts.outMin;
            }
        }

        return this;
    },
    setSampleTime   : function (newSampleTime)
    {
        if(newSampleTime > 0)
        {
            var ratio             = parseFloat(newSampleTime / this.opts.sampleTime);
            this.ki              *= ratio;
            this.kd              /= ratio;
            this.opts.sampleTime  = NewSampleTime;

            return this;
        }

        // Need to throw error
        return false;
    },
    setOutput        : function (output)
    {
        this.opts.output = output;
        return this;
    },
    setTarget       : function (target)
    {
        this.tReached    = false;
        this.opts.target = parseFloat(target);

        return this;
    },
    setTunings      : function (Kp, Ki, Kd)
    {
        if(Kp < 0 || Ki < 0 || Kd < 0)
        {
            // Should throw err here.
            return false;
        }

        var sampleTimeInSec = parseFloat(this.opts.sampleTime/1000);
        this.kp             = Kp;
        this.ki             = Ki * sampleTimeInSec;
        this.kd             = Kd / sampleTimeInSec;

        /**
         * We have a setDirection method.
         if(this.opts.direction == 1)
         {
             this.kp = (0 - this.kp);
             this.ki = (0 - this.ki);
             this.kd = (0 - this.kd);
         }
         **/

        return this;
    },
    start           : function()
    {
        // Make sure we can run run()
        this.running  = true;

        // Run-a-way intervals (timeouts)?
        clearTimeout(this.interval);

        // Safeties I guess.
        this.iTerm    = this.oVal;
        this.iValLast = this.iVal;

        if(this.iTerm > this.opts.outMax)
        {
            this.iTerm = this.opts.outMax;
        }
        else if(this.iTerm < this.opts.outMin)
        {
            this.iTerm = this.opts.outMin;
        }

        this.emit('started');
        this.run();
        return this;
    },
    status         : function()
    {
        return {'running' : this.running, 'target' : this.opts.target, 'tReached': this.tReached, 'direction' : this.opts.direction, 'duration' : this.opts.duration, 'timeLeft' : this.timeLeft};
    },
    stop            : function()
    {
        // Make sure run() can't be triggered.
        this.running  = false;

        // Stop the interval (timeout)
        clearTimeout(this.interval);

        // Set the output to 0
        this.opts.output(0);

        this.emit('stopped');
        return this;
    }
};

util.inherits(PIDController, events.EventEmitter);
module.exports    = PIDController;