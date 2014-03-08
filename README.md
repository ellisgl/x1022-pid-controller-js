PID Controller for Node.JS based on the Arduino PID library.

*Really Basic Usage:*

```
var pid      = require('x1022-pid-controller-js');
global.pCtrl = new pid({
    Kp : 1.0,
    Ki : 0.01,
    Kd : 0.01
    target: 120;
});

pCtrl.setInput(function()
{
     global.pCtrl.setIVal(getDataFromSensor());
});

pCtrl.setOutput(function(val)
{
    global.oCtrl.set(val);
});

pCtrl.start();
```

For a more complex usage, see https://github.com/ellisgl/Brewnoduino