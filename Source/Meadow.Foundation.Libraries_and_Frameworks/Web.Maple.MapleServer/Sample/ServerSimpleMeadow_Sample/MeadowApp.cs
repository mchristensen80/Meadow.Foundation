﻿using System;
using System.Threading;
using System.Threading.Tasks;
using Meadow;
using Meadow.Devices;
using Meadow.Foundation.Web.Maple;
using Meadow.Gateway.WiFi;

namespace Maple.ServerSimpleMeadow_Sample
{
    public class MeadowApp : App<F7FeatherV2>
    {
        MapleServer server;

        public override async Task Initialize()
        {
            Console.WriteLine("Initialize hardware...");
            Console.WriteLine("Initialize hardware...");

            // connnect to the wifi network.
            Console.WriteLine($"Connecting to WiFi Network {Secrets.WIFI_NAME}");
            var connectionResult = await Device.WiFiAdapter.Connect(Secrets.WIFI_NAME, Secrets.WIFI_PASSWORD);

            if (connectionResult.ConnectionStatus != ConnectionStatus.Success) 
            {
                throw new Exception($"Cannot connect to network: {connectionResult.ConnectionStatus}");
            }
            Console.WriteLine($"Connected. IP: {Device.WiFiAdapter.IpAddress}");

            // create our maple web server
            server = new MapleServer(
                Device.WiFiAdapter.IpAddress,
                advertise: true,
                processMode: RequestProcessMode.Parallel
                );
        }

        public override Task Run()
        {
            server.Start();

            return Task.CompletedTask;
        }
    }
}