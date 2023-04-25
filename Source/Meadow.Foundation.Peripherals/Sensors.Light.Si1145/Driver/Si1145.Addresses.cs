﻿namespace Meadow.Foundation.Sensors.Light
{
    public partial class Si1145
    {
        /// <summary>
        /// Valid I2C addresses for the sensor
        /// </summary>
        public enum Address : byte
        {
            /// <summary>
            /// Bus address 0x60
            /// </summary>
            Address_0x60 = 0x60,
            /// <summary>
            /// Default bus address
            /// </summary>
            Default = Address_0x60
        }
    }
}
