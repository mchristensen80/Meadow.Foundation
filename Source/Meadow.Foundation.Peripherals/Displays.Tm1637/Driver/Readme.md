# Meadow.Foundation.Displays.Tm1637

**TM1637 digital character display**

The **Tm1637** library is designed for the [Wilderness Labs](www.wildernesslabs.co) Meadow .NET IoT platform and is part of [Meadow.Foundation](https://developer.wildernesslabs.co/Meadow/Meadow.Foundation/).

The **Meadow.Foundation** peripherals library is an open-source repository of drivers and libraries that streamline and simplify adding hardware to your C# .NET Meadow IoT application.

For more information on developing for Meadow, visit [developer.wildernesslabs.co](http://developer.wildernesslabs.co/).

To view all Wilderness Labs open-source projects, including samples, visit [github.com/wildernesslabs](https://github.com/wildernesslabs/).

## Usage

```csharp
Tm1637 display;

public override Task Initialize()
{
    Resolver.Log.Info("Initialize...");

    display = new Tm1637(Device.Pins.D02, Device.Pins.D01);

    display.Brightness = 7;
    display.ScreenOn = true;

    return Task.CompletedTask;
}

public override Task Run()
{
    display.Clear();

    var chars = new Character[] { Character.A, Character.B, Character.C, Character.D };

    display.Show(chars);

    return Task.CompletedTask;
}

```
## How to Contribute

- **Found a bug?** [Report an issue](https://github.com/WildernessLabs/Meadow_Issues/issues)
- Have a **feature idea or driver request?** [Open a new feature request](https://github.com/WildernessLabs/Meadow_Issues/issues)
- Want to **contribute code?** Fork the [Meadow.Foundation](https://github.com/WildernessLabs/Meadow.Foundation) repository and submit a pull request against the `develop` branch


## Need Help?

If you have questions or need assistance, please join the Wilderness Labs [community on Slack](http://slackinvite.wildernesslabs.co/).
