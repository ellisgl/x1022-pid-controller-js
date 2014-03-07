module.exports = PIDController;

var PIDController = function(input, output, options)
{
    // Options configuration
    var defaults     = {
        target     : 0,
        sampleTime : 100,
        Kp         : 1.0,
        Ki         : 0.0,
        Kd         : 0.0,
        direction  : 0;
    };

    var options       = options || {};
    this.opts         = $.extend({}, defaults, options);

    this.input      = input; // Function (Reading from sensor)
    this.output     = output; // Function (Writting to device)
    this.iVal       = 0;
    this.iValLast   = 0;
    this.oVal       = 0;
    this.lTime      = 0;
    this.kp         = 0.0;
    this.ki         = 0.0;
    this.kd         = 0.0;
    this.dispKp     = 0.0;
    this.dispKi     = 0.0;
    this.dispKd     = 0.0;

    this.setDirection(this.opts.direction);
    this.setTunings(this.opts.Kp, this.opts.Ki, this.opts.Kd);
};

PIDController.prototype.compute = function ()
{

};

PIDController.prototype.setDirection = function (direction)
{

};
PIDController.prototype.setSampleTime = function (newSampleTime)
{

}

PIDController.prototype.setTarget = function (target)
{
    this.target = target;
};

PIDController.prototype.setTunings = function (Kp, Ki, Kd)
{
    if(Kp < 0 || Ki < 0 || Kd < 0)
    {
        // Should throw err here.
        return;
    }
};

PIDContoller.prototype.start = function()
{

}

PIDContoller.prototype.stop = function()
{
    clearInterval(this.interval);
}

module.exports = PIDController;