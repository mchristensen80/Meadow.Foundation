﻿namespace Meadow.Foundation.ICs.IOExpanders
{
    //128 LED driver
    //39 key input
    public partial class Ht16k33
    {
        /// <summary>
        /// Valid I2C addresses for the sensor
        /// </summary>
        public enum Address : byte
        {
            /// <summary>
            /// Bus address 0x70
            /// </summary>
            Address_0x70 = 0x70,
            /// <summary>
            /// Default bus address
            /// </summary>
            Default = Address_0x70
        }

        /// <summary>
        /// Blink rate
        /// </summary>
        public enum BlinkRate : byte
        {
            /// <summary>
            /// Off
            /// </summary>
            Off = 0,
            /// <summary>
            /// Fast (2Hz)
            /// </summary>
            Fast = 2, //2hz
            /// <summary>
            /// Medium (1Hz)
            /// </summary>
            Medium = 4, //1hz
            /// <summary>
            /// Slow (0.5Hz)
            /// </summary>
            Slow = 8, //0.5hz
        }

        /// <summary>
        /// Display brightness
        /// </summary>
        public enum Brightness : byte
        {
            /// <summary>
            /// 0 off
            /// </summary>
            Off = 0,
            /// <summary>
            /// Low brightness
            /// </summary>
            Low = 4,
            /// <summary>
            /// Medium brightness
            /// </summary>
            Medium = 8,
            /// <summary>
            /// High brightness
            /// </summary>
            High = 12,
            /// <summary>
            /// Maximum brightness
            /// </summary>
            Maximum = 15,
        }

        enum Register : byte
        {
            HT16K33_DSP = 128,
            HT16K33_SS = 32, // System setup register
            HT16K33_KDAP = 64, // Key Address Data Pointer
            HT16K33_IFAP = 96, // Read INT flag status
            HT16K33_DIM = 0xE0, // Set brightness / dimmer
            HT16K33_DDAP = 0, //display address pointer
        }
    }
}