# Meadow.Foundation.Sensors.Motion.Lis3mdl

**Lis3mdl I2C 3-axis magnetometer**

The **Lis3mdl** library is designed for the [Wilderness Labs](www.wildernesslabs.co) Meadow .NET IoT platform and is part of [Meadow.Foundation](https://developer.wildernesslabs.co/Meadow/Meadow.Foundation/).

The **Meadow.Foundation** peripherals library is an open-source repository of drivers and libraries that streamline and simplify adding hardware to your C# .NET Meadow IoT application.

For more information on developing for Meadow, visit [developer.wildernesslabs.co](http://developer.wildernesslabs.co/).

To view all Wilderness Labs open-source projects, including samples, visit [github.com/wildernesslabs](https://github.com/wildernesslabs/).

## Usage

```csharp
Lis3mdl sensor;

public override Task Initialize()
{
    Resolver.Log.Info("Initialize hardware...");
    sensor = new Lis3mdl(Device.CreateI2cBus());

    // classical .NET events can also be used:
    sensor.Updated += HandleResult;

    // Example that uses an IObservable subscription to only be notified when the filter is satisfied
    var consumer = Lis3mdl.CreateObserver(handler: result => HandleResult(this, result),
                                         filter: result => FilterResult(result));

    sensor.Subscribe(consumer);

    sensor.StartUpdating(TimeSpan.FromSeconds(2));

    return base.Initialize();
}

bool FilterResult(IChangeResult<MagneticField3D> result)
{
    return result.New.Z > new MagneticField(0.1, MagneticField.UnitType.Gauss);
}

void HandleResult(object sender, IChangeResult<MagneticField3D> result)
{
    var mag = result.New;

    Resolver.Log.Info($"Magnetometer (gauss): X = {mag.X.Gauss:0.##}, Y = {mag.Y.Gauss:0.##}, Z = {mag.Z.Gauss:0.##}");
}

```
## How to Contribute

- **Found a bug?** [Report an issue](https://github.com/WildernessLabs/Meadow_Issues/issues)
- Have a **feature idea or driver request?** [Open a new feature request](https://github.com/WildernessLabs/Meadow_Issues/issues)
- Want to **contribute code?** Fork the [Meadow.Foundation](https://github.com/WildernessLabs/Meadow.Foundation) repository and submit a pull request against the `develop` branch


## Need Help?

If you have questions or need assistance, please join the Wilderness Labs [community on Slack](http://slackinvite.wildernesslabs.co/).
