﻿using System;
using System.IO;
using System.Threading;
using System.Threading.Tasks;
using Meadow.Devices;
using Meadow.Foundation.Displays.TftSpi;
using Meadow.Hardware;
using Meadow.Units;

namespace Meadow.Foundation.Graphics
{
    /* 
       AISU:
        chipSelectPin: Device.Pins.D15,
        dcPin: Device.Pins.D11,
        resetPin: Device.Pins.D14, 
       JUEGO:
        chipSelectPin: Device.Pins.D14,
        dcPin: Device.Pins.D03,
        resetPin: Device.Pins.D04,
    */

    public class MeadowApp : App<F7FeatherV2>
    {
        private MicroGraphics graphics;

        public override Task Initialize()
        {
            Console.WriteLine("Initialize hardware...");

            var config = new SpiClockConfiguration(new Frequency(48000, Frequency.UnitType.Kilohertz), SpiClockConfiguration.Mode.Mode3);
            var spiBus = Device.CreateSpiBus(Device.Pins.SCK, Device.Pins.MOSI, Device.Pins.MISO, config);

            var display = new St7789(
                device: Device,
                spiBus: spiBus,
                chipSelectPin: Device.Pins.D15,
                dcPin: Device.Pins.D11,
                resetPin: Device.Pins.D14,
                width: 240, height: 240, displayColorMode: ColorType.Format16bppRgb565)
            {
            };

            graphics = new MicroGraphics(display)
            {
                Rotation = RotationType._180Degrees,
                IgnoreOutOfBoundsPixels = true
            };

            return Task.CompletedTask;
        }

        public override Task Run()
        {
            graphics.Clear();
            graphics.CurrentFont = new Font12x20();

            graphics.DrawText(5, 200, "starting...", Color.White);
            graphics.Show();
            Thread.Sleep(2000);

            while (true)
            {
                DrawImageFromFile(8);
                Thread.Sleep(2000);
                DrawImageFromResource(8);
                Thread.Sleep(2000);
                DrawImageFromFile(24);
                Thread.Sleep(2000);
                DrawImageFromResource(24);
                Thread.Sleep(2000);
            }
        }

        private void DrawImageFromFile(int depth)
        {
            Console.WriteLine("Showing file...");
            var filePath = Path.Combine(MeadowOS.FileSystem.UserFileSystemRoot, $"wl{depth}.bmp");
            var image = Image.LoadFromFile(filePath);
            graphics.Clear();
            graphics.DrawImage(image);
            graphics.DrawText(5, 200, $"{depth}bpp file", Color.White);
            graphics.Show();
        }

        private void DrawImageFromResource(int depth)
        {
            Console.WriteLine("Showing resource...");
            var image = Image.LoadFromResource($"wl{depth}_res.bmp");
            graphics.Clear();
            graphics.DrawImage(image);
            graphics.DrawText(5, 200, $"{depth}bpp resource", Color.White);
            graphics.Show();
        }
    }
}