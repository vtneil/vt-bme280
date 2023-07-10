/**
 * Modified from Tiny_BME280_Arduino_Library (https://github.com/fabyte/Tiny_BME280_Arduino_Library)
 */
#ifndef VT_BME280_H
#define VT_BME280_H

#include <Arduino.h>
#include <Wire.h>

#define NO_WIRE 0
#define BME280_DIG_T1_LSB_REG            0x88
#define BME280_DIG_T1_MSB_REG            0x89
#define BME280_DIG_T2_LSB_REG            0x8A
#define BME280_DIG_T2_MSB_REG            0x8B
#define BME280_DIG_T3_LSB_REG            0x8C
#define BME280_DIG_T3_MSB_REG            0x8D
#define BME280_DIG_P1_LSB_REG            0x8E
#define BME280_DIG_P1_MSB_REG            0x8F
#define BME280_DIG_P2_LSB_REG            0x90
#define BME280_DIG_P2_MSB_REG            0x91
#define BME280_DIG_P3_LSB_REG            0x92
#define BME280_DIG_P3_MSB_REG            0x93
#define BME280_DIG_P4_LSB_REG            0x94
#define BME280_DIG_P4_MSB_REG            0x95
#define BME280_DIG_P5_LSB_REG            0x96
#define BME280_DIG_P5_MSB_REG            0x97
#define BME280_DIG_P6_LSB_REG            0x98
#define BME280_DIG_P6_MSB_REG            0x99
#define BME280_DIG_P7_LSB_REG            0x9A
#define BME280_DIG_P7_MSB_REG            0x9B
#define BME280_DIG_P8_LSB_REG            0x9C
#define BME280_DIG_P8_MSB_REG            0x9D
#define BME280_DIG_P9_LSB_REG            0x9E
#define BME280_DIG_P9_MSB_REG            0x9F
#define BME280_DIG_H1_REG                0xA1
#define BME280_CHIP_ID_REG               0xD0 //Chip ID
#define BME280_RST_REG                   0xE0 //Softreset Reg
#define BME280_DIG_H2_LSB_REG            0xE1
#define BME280_DIG_H2_MSB_REG            0xE2
#define BME280_DIG_H3_REG                0xE3
#define BME280_DIG_H4_MSB_REG            0xE4
#define BME280_DIG_H4_LSB_REG            0xE5
#define BME280_DIG_H5_MSB_REG            0xE6
#define BME280_DIG_H6_REG                0xE7
#define BME280_CTRL_HUMIDITY_REG         0xF2 //Ctrl Humidity Reg
#define BME280_STAT_REG                  0xF3 //Status Reg
#define BME280_CTRL_MEAS_REG             0xF4 //Ctrl Measure Reg
#define BME280_CONFIG_REG                0xF5 //Configuration Reg
#define BME280_PRESSURE_MSB_REG          0xF7 //Pressure MSB
#define BME280_PRESSURE_LSB_REG          0xF8 //Pressure LSB
#define BME280_PRESSURE_XLSB_REG         0xF9 //Pressure XLSB
#define BME280_TEMPERATURE_MSB_REG       0xFA //Temperature MSB
#define BME280_TEMPERATURE_LSB_REG       0xFB //Temperature LSB
#define BME280_TEMPERATURE_XLSB_REG      0xFC //Temperature XLSB
#define BME280_HUMIDITY_MSB_REG          0xFD //Humidity MSB
#define BME280_HUMIDITY_LSB_REG          0xFE //Humidity LSB

namespace vt {
    struct bme280_settings {
        int16_t tempCorrection;
    };

    struct bme280_calibration {
        uint16_t dig_T1;
        int16_t dig_T2;
        int16_t dig_T3;

        uint16_t dig_P1;
        int16_t dig_P2;
        int16_t dig_P3;
        int16_t dig_P4;
        int16_t dig_P5;
        int16_t dig_P6;
        int16_t dig_P7;
        int16_t dig_P8;
        int16_t dig_P9;

        uint8_t dig_H1;
        int16_t dig_H2;
        uint8_t dig_H3;
        int16_t dig_H4;
        int16_t dig_H5;
        int8_t dig_H6;
    };

    enum class bme280_mode_e : uint8_t {
        SLEEP = 0b00,
        FORCED = 0b01,
        NORMAL = 0b11
    };

    class bme280_t {
    private:
        uint8_t address_i2c_ = 0x76;
        TwoWire *port_i2c_ = &Wire;
        bme280_settings settings = {};
        bme280_calibration calibration = {};
        int32_t t_fine_ = {};

    public:
        bme280_t() = default;

        uint8_t begin(uint8_t i2c_address = 0x76,
                      TwoWire *wire = &Wire,
                      uint8_t oversample_pressure = 8,
                      uint8_t oversample_humidity = 8,
                      uint8_t oversample_temperature = 8) {
            delay(2);

            address_i2c_ = i2c_address;
            port_i2c_ = wire;

            port_i2c_->begin();

            uint8_t chipID = read_register(BME280_CHIP_ID_REG); //Should return 0x60 or 0x58
            if (chipID != 0x58 && chipID != 0x60) return (chipID);

            calibration.dig_T1 = static_cast<uint16_t>((read_register(BME280_DIG_T1_MSB_REG) << 8) +
                                                       read_register(BME280_DIG_T1_LSB_REG));
            calibration.dig_T2 = static_cast<int16_t>((read_register(BME280_DIG_T2_MSB_REG) << 8) +
                                                      read_register(BME280_DIG_T2_LSB_REG));
            calibration.dig_T3 = static_cast<int16_t>((read_register(BME280_DIG_T3_MSB_REG) << 8) +
                                                      read_register(BME280_DIG_T3_LSB_REG));

            calibration.dig_P1 = static_cast<uint16_t>((read_register(BME280_DIG_P1_MSB_REG) << 8) +
                                                       read_register(BME280_DIG_P1_LSB_REG));
            calibration.dig_P2 = static_cast<int16_t>((read_register(BME280_DIG_P2_MSB_REG) << 8) +
                                                      read_register(BME280_DIG_P2_LSB_REG));
            calibration.dig_P3 = static_cast<int16_t>((read_register(BME280_DIG_P3_MSB_REG) << 8) +
                                                      read_register(BME280_DIG_P3_LSB_REG));
            calibration.dig_P4 = static_cast<int16_t>((read_register(BME280_DIG_P4_MSB_REG) << 8) +
                                                      read_register(BME280_DIG_P4_LSB_REG));
            calibration.dig_P5 = static_cast<int16_t>((read_register(BME280_DIG_P5_MSB_REG) << 8) +
                                                      read_register(BME280_DIG_P5_LSB_REG));
            calibration.dig_P6 = static_cast<int16_t>((read_register(BME280_DIG_P6_MSB_REG) << 8) +
                                                      read_register(BME280_DIG_P6_LSB_REG));
            calibration.dig_P7 = static_cast<int16_t>((read_register(BME280_DIG_P7_MSB_REG) << 8) +
                                                      read_register(BME280_DIG_P7_LSB_REG));
            calibration.dig_P8 = static_cast<int16_t>((read_register(BME280_DIG_P8_MSB_REG) << 8) +
                                                      read_register(BME280_DIG_P8_LSB_REG));
            calibration.dig_P9 = static_cast<int16_t>((read_register(BME280_DIG_P9_MSB_REG) << 8) +
                                                      read_register(BME280_DIG_P9_LSB_REG));

            calibration.dig_H1 = static_cast<uint8_t>(read_register(BME280_DIG_H1_REG));
            calibration.dig_H2 = static_cast<int16_t>((read_register(BME280_DIG_H2_MSB_REG) << 8) +
                                                      read_register(BME280_DIG_H2_LSB_REG));
            calibration.dig_H3 = static_cast<uint8_t>(read_register(BME280_DIG_H3_REG));
            calibration.dig_H4 = static_cast<int16_t>((read_register(BME280_DIG_H4_MSB_REG) << 4) +
                                                      (read_register(BME280_DIG_H4_LSB_REG) & 0x0F));
            calibration.dig_H5 = static_cast<int16_t>((read_register(BME280_DIG_H5_MSB_REG) << 4) +
                                                      ((read_register(BME280_DIG_H4_LSB_REG) >> 4) & 0x0F));
            calibration.dig_H6 = static_cast<int8_t>(read_register(BME280_DIG_H6_REG));

            set_temp_oversample(oversample_temperature);
            set_humidity_oversample(oversample_humidity);
            set_pressure_oversample(oversample_pressure);
            set_stby_time(0);
            set_filter(0);

            set_mode(bme280_mode_e::NORMAL);

            return read_register(BME280_CHIP_ID_REG);
        }

        enum bme280_mode_e get_mode() {
            uint8_t controlData = read_register(BME280_CTRL_MEAS_REG);
            return static_cast<bme280_mode_e>(controlData & 0b00000011);
        }

        void set_mode(enum bme280_mode_e mode) {
            uint8_t controlData = read_register(BME280_CTRL_MEAS_REG);
            controlData &= ~((1 << 1) | (1 << 0)); //Clear the mode[1:0] bits
            controlData |= mode; //Set
            write_register(BME280_CTRL_MEAS_REG, controlData);
        }

        void set_temp_oversample(uint8_t overSampleAmount) {
            overSampleAmount = check_sample_value(overSampleAmount);
            enum bme280_mode_e originalMode = get_mode();
            set_mode(bme280_mode_e::SLEEP);
            uint8_t controlData = read_register(BME280_CTRL_MEAS_REG);
            controlData &= ~((1 << 7) | (1 << 6) | (1 << 5));
            controlData |= overSampleAmount << 5;
            write_register(BME280_CTRL_MEAS_REG, controlData);
            set_mode(originalMode);
        }

        void set_pressure_oversample(uint8_t overSampleAmount) {
            overSampleAmount = check_sample_value(overSampleAmount);
            enum bme280_mode_e originalMode = get_mode();
            set_mode(bme280_mode_e::SLEEP);
            uint8_t controlData = read_register(BME280_CTRL_MEAS_REG);
            controlData &= ~((1 << 4) | (1 << 3) | (1 << 2));
            controlData |= overSampleAmount << 2;
            write_register(BME280_CTRL_MEAS_REG, controlData);
            set_mode(originalMode);
        }

        void set_humidity_oversample(uint8_t overSampleAmount) {
            overSampleAmount = check_sample_value(overSampleAmount);
            enum bme280_mode_e originalMode = get_mode();
            set_mode(bme280_mode_e::SLEEP);
            uint8_t controlData = read_register(BME280_CTRL_HUMIDITY_REG);
            controlData &= ~((1 << 2) | (1 << 1) | (1 << 0));
            controlData |= overSampleAmount << 0;
            write_register(BME280_CTRL_HUMIDITY_REG, controlData);
            set_mode(originalMode);
        }

        void set_stby_time(uint8_t timeSetting) {
            if (timeSetting > 0b111) timeSetting = 0;
            uint8_t controlData = read_register(BME280_CONFIG_REG);
            controlData &= ~((1 << 7) | (1 << 6) | (1 << 5));
            controlData |= (timeSetting << 5);
            write_register(BME280_CONFIG_REG, controlData);
        }

        void set_filter(uint8_t filterSetting) {
            if (filterSetting > 0b111) filterSetting = 0;
            uint8_t controlData = read_register(BME280_CONFIG_REG);
            controlData &= ~((1 << 4) | (1 << 3) | (1 << 2));
            controlData |= (filterSetting << 2);
            write_register(BME280_CONFIG_REG, controlData);
        }

        bool is_measuring() {
            uint8_t stat = read_register(BME280_STAT_REG);
            return stat & (1 << 3);
        }

        void reset() { write_register(BME280_RST_REG, 0xB6); }

        float read_pressure() { return static_cast<float>(read_raw_pressure()) * 1.e-2f; }

        float read_humidity() { return static_cast<float>(read_raw_humidity()) * 1.e-3f; }

        float read_temperature_c() { return static_cast<float>(read_raw_temp_c()) * 1.e-2f; }

        float read_temperature_f() { return static_cast<float>(read_raw_temp_f()) * 1.e-2f; }

        uint32_t read_raw_pressure() {
            uint8_t buffer[3];
            read_register_region(buffer, BME280_PRESSURE_MSB_REG, 3);
            int32_t adc_P = (int32_t) (((uint32_t) buffer[0] << 12) | ((uint32_t) buffer[1] << 4) |
                                       ((buffer[2] >> 4) & 0x0F));
            int32_t var1, var2;
            uint32_t p_acc;
            var1 = ((int32_t) t_fine_ >> 1) - (int32_t) 64000;
            var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * (int32_t) calibration.dig_P6;
            var2 = var2 + ((var1 * ((int32_t) calibration.dig_P5) << 1));
            var2 = (var2 >> 2) + (((int32_t) calibration.dig_P4) << 16);
            var1 = ((((int32_t) calibration.dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) +
                    (((((int32_t) calibration.dig_P2) * var1) >> 1))) >> 18;
            var1 = ((((32768 + var1)) * ((int32_t) calibration.dig_P1)) >> 15);
            if (var1 == 0) return 0;
            p_acc = (((uint32_t) (((int32_t) 1048576 - adc_P) - (var2 >> 12))) * 3125);
            if (p_acc < 0x80000000) p_acc = (p_acc << 1) / ((uint32_t) var1);
            else p_acc = (p_acc / (uint32_t) var1) * 2;
            var1 = (((int32_t) calibration.dig_P9) * ((int32_t) (((p_acc >> 3) * (p_acc >> 3)) >> 13))) >> 12;
            var2 = (((int32_t) (p_acc >> 2)) * ((int32_t) calibration.dig_P8)) >> 13;
            p_acc = (uint32_t) ((int32_t) p_acc + ((var1 + var2 + (int32_t) calibration.dig_P7) >> 4));
            return p_acc;
        }

        uint32_t read_raw_humidity() {
            uint8_t buffer[2];
            read_register_region(buffer, BME280_HUMIDITY_MSB_REG, 2);
            int32_t adc_H = (int32_t) (((uint32_t) buffer[0] << 8) | ((uint32_t) buffer[1]));
            int32_t var1;
            var1 = (t_fine_ - ((int32_t) 76800));
            var1 = (((
                    ((adc_H << 14) - (((int32_t) calibration.dig_H4) << 20) - (((int32_t) calibration.dig_H5) * var1)) +
                    ((int32_t) 16384)) >> 15) * (((((((var1 * ((int32_t) calibration.dig_H6)) >> 10) *
                                                     (((var1 * ((int32_t) calibration.dig_H3)) >> 11) +
                                                      ((int32_t) 32768))) >> 10) + ((int32_t) 2097152)) *
                                                  ((int32_t) calibration.dig_H2) + 8192) >> 14));
            var1 = (var1 - (((((var1 >> 15) * (var1 >> 15)) >> 7) * ((int32_t) calibration.dig_H1)) >> 4));
            var1 = (var1 < 0 ? 0 : var1);
            var1 = (var1 > 419430400 ? 419430400 : var1);
            return (uint32_t) (var1 >> 12) * 1000 / 1024;
        }

        int32_t read_raw_temp_c() {
            uint8_t buffer[3];
            read_register_region(buffer, BME280_TEMPERATURE_MSB_REG, 3);
            int32_t adc_T = (int32_t) (((uint32_t) buffer[0] << 12) | ((uint32_t) buffer[1] << 4) |
                                       ((buffer[2] >> 4) & 0x0F));
            int32_t var1, var2;
            var1 = ((((adc_T >> 3) - ((int32_t) calibration.dig_T1 << 1))) * ((int32_t) calibration.dig_T2)) >> 11;
            var2 = (((((adc_T >> 4) - ((int32_t) calibration.dig_T1)) * ((adc_T >> 4) - ((int32_t) calibration.dig_T1)))
                    >> 12) *
                    ((int32_t) calibration.dig_T3)) >> 14;
            t_fine_ = var1 + var2;
            return ((t_fine_ * 5 + 128) >> 8) + (settings.tempCorrection);
        }

        int32_t read_raw_temp_f() { return (read_raw_temp_c() * 9) / 5 + 3200; }

    private:
        static uint8_t check_sample_value(uint8_t userValue) {
            switch (userValue) {
                case (0):
                    return 0;
                case (2):
                    return 2;
                case (4):
                    return 3;
                case (8):
                    return 4;
                case (16):
                    return 5;
                case (1): // Fall through
                default:
                    return 1;
            }
        }

        void read_register_region(uint8_t *outputPointer, uint8_t offset, uint8_t length) {
            uint8_t i = 0;
            char c;
            port_i2c_->beginTransmission(address_i2c_);
            port_i2c_->write(offset);
            port_i2c_->endTransmission();
            port_i2c_->requestFrom(address_i2c_, length);
            while ((port_i2c_->available()) && (i < length)) {
                c = port_i2c_->read();
                *outputPointer = c;
                ++outputPointer;
                ++i;
            }
        }

        uint8_t read_register(uint8_t offset) {
            uint8_t result = 0;
            uint8_t numBytes = 1;
            port_i2c_->beginTransmission(address_i2c_);
            port_i2c_->write(offset);
            port_i2c_->endTransmission();

            port_i2c_->requestFrom(address_i2c_, numBytes);
            while (port_i2c_->available()) result = port_i2c_->read();
            return result;
        }

        void write_register(uint8_t offset, uint8_t dataToWrite) {
            port_i2c_->beginTransmission(address_i2c_);
            port_i2c_->write(offset);
            port_i2c_->write(dataToWrite);
            port_i2c_->endTransmission();
        }
    };
}

#endif //VT_BME280_H
