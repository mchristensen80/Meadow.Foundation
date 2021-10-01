﻿using System;
using Meadow.Hardware;

namespace Meadow.Foundation.ICs.IOExpanders
{
    //128 LED driver
    //39 key input
    public partial class Ht16k33
    {
        /// <summary>
        ///     HT16K33 LED driver and key scan
        /// </summary>
        private readonly II2cPeripheral i2cPeripheral;

        //display buffer for 16x8 LEDs
        private byte[] displayBuffer = new byte[16];
        //key buffer for 39 keys
        private byte[] keyBuffer = new byte[6];

        readonly byte HT16K33_DSP = 128;
        readonly byte HT16K33_SS = 32; // System setup register
        readonly byte HT16K33_KDAP = 64; // Key Address Data Pointer
        readonly byte HT16K33_IFAP = 96; // Read INT flag status
        readonly byte HT16K33_DIM = 0xE0; // Set brightness / dimmer
        readonly byte HT16K33_DDAP = 0; //display address pointer

        /// <summary>
        ///     Create a new HT16K33 object using the default parameters
        /// </summary>
        /// <param name="address">Address of the bus on the I2C display.</param>
        /// <param name="i2cBus">I2C bus instance</param>
        public Ht16k33(II2cBus i2cBus, byte address = (byte)Addresses.Default)
        {
            i2cPeripheral = new I2cPeripheral(i2cBus, address);

            InitHT16K33();
        }

        void InitHT16K33()
        {
            //   SetIsAwake(true);
            //   SetDisplayOn(true);
            //   SetBlinkRate(BlinkRate.Off);

            i2cPeripheral.Write(0x21);

            i2cPeripheral.Write(0x81);

            SetBrightness(Brightness.Maximum);
            ClearDisplay();
        }

        public void SetIsAwake(bool awake)
        {
            byte value = (byte)(HT16K33_SS | (byte)(awake ? 1 : 0));

            i2cPeripheral.Write(value);
        }

        public void SetDisplayOn(bool isOn)
        {
            byte value = (byte)(HT16K33_DSP | (byte)(isOn ? 1 : 0));

            i2cPeripheral.Write(value);
        }

        public void SetBlinkRate(BlinkRate blinkRate)
        {
            byte value = (byte)(HT16K33_DSP | (byte)blinkRate);

            i2cPeripheral.Write(value);
        }

        public void SetBrightness(Brightness brightness)
        {
            byte value = (byte)(HT16K33_DIM | (byte)brightness);

            i2cPeripheral.Write(value);
        }

        public void ClearDisplay()
        {
            for (int i = 0; i < displayBuffer.Length; i++)
                displayBuffer[i] = 0;

            UpdateDisplay();
        }

        public void UpdateDisplay()
        {
            i2cPeripheral.WriteRegisters(0x0, displayBuffer);
        }

        public void SetLed(byte ledIndex, bool ledOn)
        {
            if (ledIndex > 127)
            {
                throw new IndexOutOfRangeException("LED Index must be between 0 and 127");
            }

            var index = ledIndex / 8;

            if (ledOn)
            {
                displayBuffer[index] = (byte)(displayBuffer[index] | (byte)(1 << (ledIndex % 8)));
            }
            else
            {
                displayBuffer[index] = (byte)(displayBuffer[index] & ~(byte)(1 << (ledIndex % 8)));
            }
        }

        public void ToggleLed(byte ledIndex)
        {
            var index = ledIndex / 8;

            displayBuffer[index] = (displayBuffer[index] ^= (byte)(1 << ledIndex % 8));
        }

        public bool IsLedOn(int ledIndex)
        {
            //need to do some bit math here
            var index = ledIndex / 8;

            //untested
            return displayBuffer[index] >> ledIndex != 0;
        }
    }
}