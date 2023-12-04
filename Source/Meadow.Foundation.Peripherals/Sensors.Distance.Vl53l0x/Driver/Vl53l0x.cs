using Meadow.Hardware;
using Meadow.Peripherals.Sensors.Distance;
using Meadow.Units;
using System;
using System.Threading;
using System.Threading.Tasks;

// TODO: Does this driver support the three modes of operations? High accuracy, long range, high speed?
// If not, then we should add a property and/or constructor parameter to select the mode of operation.

// TODO: Research the git commit history on this class in the repository. What issues were fixed?

// There are 4 range profiles available via API example code. I've removed the error checks for brevity.
// 7.1 High accuracy
// The following settings have to be applied, before ranging:
//     Status = VL53L0_SetLimitCheckValue(pMyDevice, VL53L0_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, (FixPoint1616_t)(0.25 * 65536));
//     Status = VL53L0_SetLimitCheckValue(pMyDevice, VL53L0_CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t)(18 * 65536));
//     Status = VL53L0_SetMeasurementTimingBudgetMicroSeconds(pMyDevice, 200000);
// 7.2 Long range
// The following setting have to be applied, before ranging:
//     Status = VL53L0_SetLimitCheckValue(pMyDevice, VL53L0_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, (FixPoint1616_t)(0.1 * 65536));
//     Status = VL53L0_SetLimitCheckValue(pMyDevice, VL53L0_CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t)(60 * 65536));
//     Status = VL53L0_SetMeasurementTimingBudgetMicroSeconds(pMyDevice, 33000);
//     Status = VL53L0_SetVcselPulsePeriod(pMyDevice, VL53L0_VCSEL_PERIOD_PRE_RANGE, 18);
//     Status = VL53L0_SetVcselPulsePeriod(pMyDevice, VL53L0_VCSEL_PERIOD_FINAL_RANGE, 14);
// 7.3 High speed
// The following setting have to be applied, before ranging:
//     Status = VL53L0_SetLimitCheckValue(pMyDevice, VL53L0_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, (FixPoint1616_t)(0.25 * 65536));
//     Status = VL53L0_SetLimitCheckValue(pMyDevice, VL53L0_CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t)(32 * 65536));
//     Status = VL53L0_SetMeasurementTimingBudgetMicroSeconds(pMyDevice, 20000);

// Table 6. Example API range profiles
// ||              |Timing budget |Typical max range   |Typical application                       ||
// ||Default mode  |30ms          |1.2m (white target) |standard                                  ||
// ||High accuracy |200ms         |1.2m (white target) |precise measurement                       ||
// ||Long range    |33ms          |2m (white target)   |long ranging, only for dark conditions    ||
// ||High Speed    |20ms          |1.2m (white target) |high speed where accuracy is not priority ||

namespace Meadow.Foundation.Sensors.Distance
{
    /// <summary>
    /// Represents the Vl53l0x distance sensor
    /// </summary>
    public partial class Vl53l0x : ByteCommsSensorBase<Length>, IRangeFinder, II2cPeripheral
    {
        /// <summary>
        /// Distance updated event
        /// </summary>
        public event EventHandler<IChangeResult<Length>> DistanceUpdated = default!;

        /// <summary>
        /// Is the hardware shutdown / off
        /// </summary>
        public bool IsShutdown
        {
            get
            {
                if (shutdownPort != null)
                {
                    return !shutdownPort.State;
                }
                else
                {
                    return false;
                }
            }
        }

        /// <summary>
        /// The distance to the measured object
        /// </summary>
        public Length? Distance { get; protected set; } = new Length(0);

        /// <summary>
        /// Minimum valid distance
        /// </summary>
        public Length MinimumDistance => new Length(30, Length.UnitType.Millimeters);

        /// <summary>
        /// Maximum valid distance (CurrentDistance returns -1 if above)
        /// </summary>
        public Length MaximumDistance => new Length(2000, Length.UnitType.Millimeters);

        /// <summary>
        /// The default I2C address for the peripheral
        /// </summary>
        public byte DefaultI2cAddress => (byte)Addresses.Default;

        readonly IDigitalOutputPort? shutdownPort;

        byte stopVariable;

        /// <summary>
        /// Creates a new Vl53l0x object
        /// </summary>
        /// <param name="i2cBus">I2C bus</param>
        /// <param name="address">I2C address</param>
        public Vl53l0x(II2cBus i2cBus, byte address = (byte)Addresses.Default)
                : this(i2cBus, null, address)
        { }

        /// <summary>
        /// Creates a new Vl53l0x object
        /// </summary>
        /// <param name="i2cBus">I2C bus</param>
        /// <param name="shutdownPin">Shutdown pin</param>
        /// <param name="address">VL53L0X address</param>

        public Vl53l0x(II2cBus i2cBus, IPin? shutdownPin, byte address = (byte)Addresses.Default)
                : base(i2cBus, address)
        {
            if (shutdownPin != null)
            {
                shutdownPort = shutdownPin?.CreateDigitalOutputPort(true);
            }
            Initialize().Wait();
        }

        /// <summary>
        /// Raise distance change event and notify subscribers
        /// </summary>
        /// <param name="changeResult"></param>
        protected override void RaiseEventsAndNotify(IChangeResult<Length> changeResult)
        {
            DistanceUpdated?.Invoke(this, changeResult);
            base.RaiseEventsAndNotify(changeResult);
        }

        /// <summary>
        /// Initializes the VL53L0X
        /// </summary>
        protected async Task Initialize(bool io_2v8 = true)
        {
            if (IsShutdown)
            {
                await ShutDown(false);
            }

            if (Read(0xC0) != 0xEE || Read(0xC1) != 0xAA || Read(0xC2) != 0x10)
            {
                throw new Exception("Failed to find expected ID register values");
            }

            // sensor uses 1V8 mode for I/O by default; switch to 2V8 mode if necessary
            // https://community.st.com/t5/imaging-sensors/vl53l1x-what-is-the-difference-between-2v8-and-1v8-mode/td-p/277438
            if (io_2v8)
            {
                BusComms.WriteRegister(0x89, (byte)(Read(0x89) | 0x01));
            }

            BusComms.WriteRegister(0x88, 0x00);
            BusComms.WriteRegister(0x80, 0x01);
            BusComms.WriteRegister(0xFF, 0x01);
            BusComms.WriteRegister(0x00, 0x00);

            stopVariable = Read(0x91);

            BusComms.WriteRegister(0x00, 0x01);
            BusComms.WriteRegister(0xFF, 0x00);
            BusComms.WriteRegister(0x80, 0x00);

            // TODO: do we need to disable the signal rate msrc and signal rate pre range limit checks?

            // From https://github.com/pololu/vl53l0x-arduino/blob/master/VL53L0X.cpp
            // // disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4) limit checks
            // writeReg(MSRC_CONFIG_CONTROL, readReg(MSRC_CONFIG_CONTROL) | 0x12);

            // // set final range signal rate limit to 0.25 MCPS (million counts per second)
            // setSignalRateLimit(0.25);

            BusComms.WriteRegister((byte)Register.SystemSequenceConfig, 0xFF);

            // TOOD: double check the spad info

            var spadInfo = GetSpadInfo();
            int spadCount = spadInfo.Item1;
            bool spad_is_aperture = spadInfo.Item2;

            byte[] ref_spad_map = new byte[7];
            ref_spad_map[0] = (byte)Register.GlobalConfigSpadEnablesRef0;

            BusComms.WriteRegister(0xFF, 0x01);
            BusComms.WriteRegister((byte)Register.DynamicSpadRefEnStartOffset, 0x00);
            BusComms.WriteRegister((byte)Register.DynamicSpadNumRequestedRefSpad, 0x2C);
            BusComms.WriteRegister(0xFF, 0x00);
            BusComms.WriteRegister((byte)Register.GlobalConfigRefEnStartSelect, 0xB4);

            var first_spad_to_enable = (spad_is_aperture) ? 12 : 0;
            var spads_enabled = 0;

            for (int i = 0; i < 48; i++)
            {
                if (i < first_spad_to_enable || spads_enabled == spadCount)
                {
                    ref_spad_map[1 + (i / 8)] &= (byte)~(1 << (i % 8));
                }
                else if ((ref_spad_map[1 + (i / 8)] >> (byte)((i % 8)) & 0x1) > 0)
                {
                    spads_enabled += 1;
                }
            }

            BusComms.WriteRegister(0xFF, 0x01);
            BusComms.WriteRegister(0x00, 0x00);

            BusComms.WriteRegister(0xFF, 0x00);
            BusComms.WriteRegister(0x09, 0x00);
            BusComms.WriteRegister(0x10, 0x00);
            BusComms.WriteRegister(0x11, 0x00);

            BusComms.WriteRegister(0x24, 0x01);
            BusComms.WriteRegister(0x25, 0xFF);
            BusComms.WriteRegister(0x75, 0x00);

            BusComms.WriteRegister(0xFF, 0x01);
            BusComms.WriteRegister(0x4E, 0x2C);
            BusComms.WriteRegister(0x48, 0x00);
            BusComms.WriteRegister(0x30, 0x20);

            BusComms.WriteRegister(0xFF, 0x00);
            BusComms.WriteRegister(0x30, 0x09);
            BusComms.WriteRegister(0x54, 0x00);
            BusComms.WriteRegister(0x31, 0x04);
            BusComms.WriteRegister(0x32, 0x03);
            BusComms.WriteRegister(0x40, 0x83);
            BusComms.WriteRegister(0x46, 0x25);
            BusComms.WriteRegister(0x60, 0x00);
            BusComms.WriteRegister(0x27, 0x00);
            BusComms.WriteRegister(0x50, 0x06);
            BusComms.WriteRegister(0x51, 0x00);
            BusComms.WriteRegister(0x52, 0x96);
            BusComms.WriteRegister(0x56, 0x08);
            BusComms.WriteRegister(0x57, 0x30);
            BusComms.WriteRegister(0x61, 0x00);
            BusComms.WriteRegister(0x62, 0x00);
            BusComms.WriteRegister(0x64, 0x00);
            BusComms.WriteRegister(0x65, 0x00);
            BusComms.WriteRegister(0x66, 0xA0);

            BusComms.WriteRegister(0xFF, 0x01);
            BusComms.WriteRegister(0x22, 0x32);
            BusComms.WriteRegister(0x47, 0x14);
            BusComms.WriteRegister(0x49, 0xFF);
            BusComms.WriteRegister(0x4A, 0x00);
            
            BusComms.WriteRegister(0xFF, 0x00);
            BusComms.WriteRegister(0x7A, 0x0A);
            BusComms.WriteRegister(0x7B, 0x00);
            BusComms.WriteRegister(0x78, 0x21);
            
            BusComms.WriteRegister(0xFF, 0x01);
            BusComms.WriteRegister(0x23, 0x34);
            BusComms.WriteRegister(0x42, 0x00);
            BusComms.WriteRegister(0x44, 0xFF);
            BusComms.WriteRegister(0x45, 0x26);
            BusComms.WriteRegister(0x46, 0x05);
            BusComms.WriteRegister(0x40, 0x40);
            BusComms.WriteRegister(0x0E, 0x06);
            BusComms.WriteRegister(0x20, 0x1A);
            BusComms.WriteRegister(0x43, 0x40);
            
            BusComms.WriteRegister(0xFF, 0x00);
            BusComms.WriteRegister(0x34, 0x03);
            BusComms.WriteRegister(0x35, 0x44);
            
            BusComms.WriteRegister(0xFF, 0x01);
            BusComms.WriteRegister(0x31, 0x04);
            BusComms.WriteRegister(0x4B, 0x09);
            BusComms.WriteRegister(0x4C, 0x05);
            BusComms.WriteRegister(0x4D, 0x04);
            
            BusComms.WriteRegister(0xFF, 0x00);
            BusComms.WriteRegister(0x44, 0x00);
            BusComms.WriteRegister(0x45, 0x20);
            BusComms.WriteRegister(0x47, 0x08);
            BusComms.WriteRegister(0x48, 0x28);
            BusComms.WriteRegister(0x67, 0x00);
            BusComms.WriteRegister(0x70, 0x04);
            BusComms.WriteRegister(0x71, 0x01);
            BusComms.WriteRegister(0x72, 0xFE);
            BusComms.WriteRegister(0x76, 0x00);
            BusComms.WriteRegister(0x77, 0x00);
            
            BusComms.WriteRegister(0xFF, 0x01);
            BusComms.WriteRegister(0x0D, 0x01);
            
            BusComms.WriteRegister(0xFF, 0x00);
            BusComms.WriteRegister(0x80, 0x01);
            BusComms.WriteRegister(0x01, 0xF8);
            
            BusComms.WriteRegister(0xFF, 0x01);
            BusComms.WriteRegister(0x8E, 0x01);
            BusComms.WriteRegister(0x00, 0x01);
            BusComms.WriteRegister(0xFF, 0x00);
            BusComms.WriteRegister(0x80, 0x00);

            BusComms.WriteRegister((byte)Register.SystemInterruptConfigGpio, 0x04);
            var gpio_hv_mux_active_high = Read((byte)Register.GpioHvMuxActiveHigh);
            BusComms.WriteRegister((byte)Register.GpioHvMuxActiveHigh, (byte)(gpio_hv_mux_active_high & ~0x10));

            // TODO: shouldn't this be system interrupt clear, 0x0B? It looks like it's overwriting the previous statement
            // BusComms.WriteRegister((byte)Register.GpioHvMuxActiveHigh, 0x01);
            BusComms.WriteRegister((byte)Register.SystemInterruptClear, 0x01);

            BusComms.WriteRegister((byte)Register.SystemSequenceConfig, 0xE8);

            BusComms.WriteRegister((byte)Register.SystemSequenceConfig, 0x01);
            
            PerformSingleRefCalibration(0x40);
            
            BusComms.WriteRegister((byte)Register.SystemSequenceConfig, 0x02);
            
            PerformSingleRefCalibration(0x00);

            BusComms.WriteRegister((byte)Register.SystemSequenceConfig, 0xE8);
        }

        /// <summary>
        /// Tell the sensor to take a measurement
        /// </summary>
        public void MeasureDistance()
        {
            _ = ReadSensor();
        }

        /// <summary>
        /// Returns the current distance/range
        /// </summary>
        /// <returns>The distance in the specified Units. Default mm. Returns -1 if the shutdown pin is used and is off</returns>
        protected override async Task<Length> ReadSensor()
        {
            //Resolver.Log.Info("ReadSensor");

            if (IsShutdown)
            {
                return new Length(-1f, Length.UnitType.Millimeters);
            }

            // get the distance
            Distance = new Length(await GetRawRangeData(), Length.UnitType.Millimeters);

            // throw away invalid distances if out of range
            if (Distance > MaximumDistance)
            {
                Distance = new Length(-1, Length.UnitType.Millimeters);
            }

            return Distance.Value;
        }

        /// <summary>
        /// Set a new I2C address
        /// </summary>
        /// <param name="newAddress"></param>
        public void SetAddress(byte newAddress)
        {
            if (IsShutdown)
                return;

            byte address = (byte)(newAddress & 0x7F);
            BusComms.WriteRegister((byte)Register.I2CSlaveDeviceAddress, address);
        }

        /// <summary>
        /// Set the Shutdown state of the device
        /// </summary>
        /// <param name="state">true = off/shutdown. false = on</param>
        public async Task ShutDown(bool state)
        {
            if (shutdownPort == null)
            {
                return;
            }

            shutdownPort.State = !state;
            await Task.Delay(2).ConfigureAwait(false);

            if (state == false)
            {
                await Initialize();
                // TODO: is this still needed? the previous line wasn't awaited before.
                await Task.Delay(2).ConfigureAwait(false);
            }
        }

        /// <summary>
        /// Get the spad info
        /// </summary>
        /// <returns></returns>
        /// <exception cref="Exception"></exception>
        protected Tuple<int, bool> GetSpadInfo()
        {
            BusComms.WriteRegister(0x80, 0x01);
            BusComms.WriteRegister(0xFF, 0x01);
            BusComms.WriteRegister(0x00, 0x00);
            BusComms.WriteRegister(0xFF, 0x06);

            var result = (byte)(Read(0x83) | 0x04);
            BusComms.WriteRegister(0x83, result);

            BusComms.WriteRegister(0xFF, 0x07);
            BusComms.WriteRegister(0x81, 0x01);
            BusComms.WriteRegister(0x94, 0x6B);

            BusComms.WriteRegister(0x83, 0x00);

            int tCount = 0;
            while (Read(0x83) == 0x00)
            {
                Thread.Sleep(5);
                tCount++;
                if (tCount > 100)
                {
                    throw new Exception("Timeout");
                }
            }

            BusComms.WriteRegister(0x83, 0x01);
            var tmp = Read(0x92);
            var count = tmp & 0x7F;
            var is_aperture = ((tmp >> 7) & 0x01) == 1;

            BusComms.WriteRegister(0x81, 0x00);
            BusComms.WriteRegister(0xFF, 0x06);

            var t = (byte)(Read(0x83) & ~0x04);
            BusComms.WriteRegister(0xFF, t);

            BusComms.WriteRegister(0xFF, 0x01);
            BusComms.WriteRegister(0x00, 0x01);
            BusComms.WriteRegister(0xFF, 0x00);
            BusComms.WriteRegister(0x80, 0x00);

            return new Tuple<int, bool>(count, is_aperture);
        }

        /// <summary>
        /// perform a sensor self calibration
        /// </summary>
        /// <param name="vhvInitByte">The VHV init byte</param>
        /// <exception cref="Exception"></exception>
        protected void PerformSingleRefCalibration(byte vhvInitByte)
        {
//            BusComms.WriteRegister((byte)Register.RangeStart, (byte)(0x01 | vhvInitByte & 0xFF));
// following https://github.com/pololu/vl53l0x-arduino/blob/master/VL53L0X.cpp
            BusComms.WriteRegister((byte)Register.RangeStart, (byte)(0x01 | vhvInitByte));

            int tCount = 0;

            while ((byte)(Read((byte)Register.ResultInterruptStatus) & 0x07) == 0)
            {
                Thread.Sleep(5);
                tCount++;
                if (tCount > 100)
                {
                    throw new Exception("Interrupt Status Timeout");
                }
            }

// following https://github.com/pololu/vl53l0x-arduino/blob/master/VL53L0X.cpp
//            BusComms.WriteRegister((byte)Register.GpioHvMuxActiveHigh, 0x01);
            BusComms.WriteRegister((byte)Register.SystemInterruptClear, 0x01);

            BusComms.WriteRegister((byte)Register.RangeStart, 0x00);
        }

        byte Read(byte address)
        {
            var result = BusComms.ReadRegister(address);
            return result;
        }

        int Read16(byte address)
        {
            //var result = BusComms.ReadRegisters(address, 2);
            BusComms.ReadRegister(address, ReadBuffer.Span[0..2]);
            return (ReadBuffer.Span[0] << 8) | ReadBuffer.Span[1];
        }

        async Task<int> GetRawRangeData()
        {
            BusComms.WriteRegister(0x80, 0x01);
            BusComms.WriteRegister(0xFF, 0x01);
            BusComms.WriteRegister(0x00, 0x00);
            BusComms.WriteRegister(0x91, stopVariable);
            BusComms.WriteRegister(0x00, 0x01);
            BusComms.WriteRegister(0xFF, 0x00);
            BusComms.WriteRegister(0x80, 0x00);
            BusComms.WriteRegister((byte)Register.RangeStart, 0x01);

            int tCount = 0;
            while ((byte)(Read((byte)Register.RangeStart) & 0x01) > 0)
            {
                await Task.Delay(5).ConfigureAwait(false);

                tCount++;
                if (tCount > 100)
                {
                    throw new Exception("VL53L0X Range Start Timeout");
                }
            }

            tCount = 0;
            while ((byte)(Read((byte)Register.ResultInterruptStatus) & 0x07) == 0)
            {
                await Task.Delay(5).ConfigureAwait(false);

                tCount++;
                if (tCount > 100)
                {
                    throw new Exception("VL53L0X Interrupt Status Timeout");
                }
            }

            var range_mm = Read16((byte)Register.ResultRangeStatus + 10);

// Following guidance from https://github.com/pololu/vl53l0x-arduino/blob/master/VL53L0X.cpp#L813
//            BusComms.WriteRegister((byte)Register.GpioHvMuxActiveHigh, 0x01);
            BusComms.WriteRegister((byte)Register.SystemInterruptClear, 0x01);

            return range_mm;
        }

        /// <summary>
        /// Shut down/power down the sensor
        /// </summary>
        public void ShutDown()
        {
            if (shutdownPort != null)
            {
                shutdownPort.State = true;
            }
        }
    }
}