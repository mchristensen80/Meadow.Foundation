﻿using System;
using System.Threading.Tasks;
using Meadow;
using Meadow.Devices;
using Meadow.Foundation.Sensors.Atmospheric;

namespace Sensors.Atmospheric.Bmp180_Sample
{
    public class MeadowApp : App<F7Micro, MeadowApp>
    {
        Bmp180 sensor;

        public MeadowApp()
        {
            Console.WriteLine("Initializing...");

            // configure our sensor on the I2C Bus
            sensor = new Bmp180(Device.CreateI2cBus());

            //==== IObservable 
            // Example that uses an IObersvable subscription to only be notified
            // when the temperature changes by at least a degree, and humidty by 5%.
            // (blowing hot breath on the sensor should trigger)
            var consumer = Bmp180.CreateObserver(
                handler: result => {
                    Console.WriteLine($"Observer: Temp changed by threshold; new temp: {result.New.Temperature?.Celsius:N2}C, old: {result.Old?.Temperature?.Celsius:N2}C");
                },
                // only notify if the change is greater than 0.5°C
                filter: result => {
                    if (result.Old is { } old) { //c# 8 pattern match syntax. checks for !null and assigns var.
                        return (
                        (result.New.Temperature.Value - old.Temperature.Value).Abs().Celsius > 0.5); // returns true if > 0.5°C change.
                    }
                    return false;
                }
                // if you want to always get notified, pass null for the filter:
                //filter: null
                );
            sensor.Subscribe(consumer);

            //==== Events
            // classical .NET events can also be used:
            sensor.Updated += (object sender, IChangeResult <(Meadow.Units.Temperature? Temperature, Meadow.Units.Pressure? Pressure)> e) => {
                Console.WriteLine($"  Temperature: {e.New.Temperature?.Celsius:N2}C");
                Console.WriteLine($"  Pressure: {e.New.Pressure?.Bar:N2}bar");
            };

            //==== one-off read
            ReadConditions().Wait();

            // start updating continuously
            sensor.StartUpdating(TimeSpan.FromSeconds(1));
        }

        protected async Task ReadConditions()
        {
            var conditions = await sensor.Read();
            Console.WriteLine($"Temperature: {conditions.Temperature?.Celsius}°C, Pressure: {conditions.Pressure?.Pascal}Pa");
        }
    }
}