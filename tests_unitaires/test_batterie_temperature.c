/* Tests des drivers INA226 (batterie) et DS18B20 (température). */
#include "mini_tests.h"
#include "ina226.h"
#include "ds18b20.h"
#include "faux_composants.h"
#include <math.h>

/* ================================================================== INA226 */
static void test_ina_init_checks_ids_and_writes_config(void)
{
    fake_ina226_t f; ina226_t s;
    fake_ina226_setup(&f);
    MU_ASSERT_STATUS(SENS_OK, ina226_init(&s, 0x40, 0.010f, INA226_AVG_16));
    MU_ASSERT_INT(0x4527, f.last_config);              /* 0x4127 avec AVG=16 (010) dans les bits 11:9 */
    MU_ASSERT_STATUS(SENS_OK, ina226_init(&s, 0x40, 0.010f, INA226_AVG_1));
    MU_ASSERT_INT(0x4127, f.last_config);
}

static void test_ina_init_errors(void)
{
    fake_ina226_t f; ina226_t s;
    fake_ina226_setup(&f);
    f.regs[0xFE] = 0x1234;                             /* mauvais fabricant */
    MU_ASSERT_STATUS(SENS_ERR_ID, ina226_init(&s, 0x40, 0.010f, INA226_AVG_16));
    f.regs[0xFE] = 0x5449; f.regs[0xFF] = 0x0000;      /* mauvais identifiant de puce */
    MU_ASSERT_STATUS(SENS_ERR_ID, ina226_init(&s, 0x40, 0.010f, INA226_AVG_16));
    f.regs[0xFF] = 0x2260;
    MU_ASSERT_STATUS(SENS_ERR_PARAM, ina226_init(&s, 0x40, 0.0f, INA226_AVG_16));
    MU_ASSERT_STATUS(SENS_ERR_PARAM, ina226_init(&s, 0x40, -0.01f, INA226_AVG_16));
    MU_ASSERT_STATUS(SENS_ERR_PARAM, ina226_init(NULL, 0x40, 0.01f, INA226_AVG_16));
    fake_i2c_set_fail(true);
    MU_ASSERT_STATUS(SENS_ERR_BUS, ina226_init(&s, 0x40, 0.010f, INA226_AVG_16));
}

static void test_ina_read_voltage_current_power(void)
{
    fake_ina226_t f; ina226_t s; ina226_data_t d;
    fake_ina226_setup(&f);
    ina226_init(&s, 0x40, 0.010f, INA226_AVG_16);
    fake_ina226_set(&f, 12.30f, 5.0f);                 /* 5 mV aux bornes de 10 mohm => 0,5 A */
    MU_ASSERT_STATUS(SENS_OK, ina226_read(&s, &d));
    MU_ASSERT_NEAR(12.30, d.bus_v, 0.002);
    MU_ASSERT_NEAR(5.0, d.shunt_mv, 0.003);
    MU_ASSERT_NEAR(0.5, d.current_a, 0.001);
    MU_ASSERT_NEAR(6.15, d.power_w, 0.02);
    MU_ASSERT(!d.saturated);
}

static void test_ina_negative_current_and_other_shunt_value(void)
{
    fake_ina226_t f; ina226_t s; ina226_data_t d;
    fake_ina226_setup(&f);
    ina226_init(&s, 0x40, 0.002f, INA226_AVG_16);      /* shunt de 2 mohm */
    fake_ina226_set(&f, 12.6f, -8.0f);                 /* -8 mV => -4 A (recharge) */
    MU_ASSERT_STATUS(SENS_OK, ina226_read(&s, &d));
    MU_ASSERT_NEAR(-4.0, d.current_a, 0.002);
    MU_ASSERT_NEAR(-50.4, d.power_w, 0.05);
}

static void test_ina_saturation_flag_and_bus_msb_masked(void)
{
    fake_ina226_t f; ina226_t s; ina226_data_t d;
    fake_ina226_setup(&f);
    ina226_init(&s, 0x40, 0.010f, INA226_AVG_16);
    f.regs[0x01] = 0x7FFF;                              /* convertisseur en butée haute */
    MU_ASSERT_STATUS(SENS_OK, ina226_read(&s, &d));
    MU_ASSERT(d.saturated);
    MU_ASSERT_NEAR(8.19, d.current_a, 0.01);
    f.regs[0x01] = 0x8000;                              /* butée basse */
    ina226_read(&s, &d);
    MU_ASSERT(d.saturated);
    f.regs[0x01] = 0x0010;
    f.regs[0x02] = (uint16_t)(0x8000 | 9600);           /* bit 15 parasite ignoré : 12,0 V */
    ina226_read(&s, &d);
    MU_ASSERT(!d.saturated);
    MU_ASSERT_NEAR(12.0, d.bus_v, 0.002);
}

static void test_ina_read_bus_error(void)
{
    fake_ina226_t f; ina226_t s; ina226_data_t d;
    fake_ina226_setup(&f);
    ina226_init(&s, 0x40, 0.010f, INA226_AVG_16);
    fake_remove_i2c(0x40);
    MU_ASSERT_STATUS(SENS_ERR_BUS, ina226_read(&s, &d));
}

/* ================================================================== DS18B20 */
static void test_ds18b20_crc8_known_vectors(void)
{
    const uint8_t rom[7] = { 0x02, 0x1C, 0xB8, 0x01, 0x00, 0x00, 0x00 };   /* exemple de la note d'application Maxim AN27 */
    MU_ASSERT_INT(0xA2, ds18b20_crc8(rom, 7));
    uint8_t block[8] = { 0x02, 0x1C, 0xB8, 0x01, 0x00, 0x00, 0x00, 0xA2 };
    MU_ASSERT_INT(0x00, ds18b20_crc8(block, 8));                              /* bloc + CRC => 0 */
    block[3] ^= 0x01;
    MU_ASSERT(ds18b20_crc8(block, 8) != 0);                                   /* un bit faux est détecté */
    MU_ASSERT_INT(0x00, ds18b20_crc8(rom, 0));
}

static void test_ds18b20_raw_conversion_datasheet_values(void)
{
    MU_ASSERT_NEAR(125.0, ds18b20_raw_to_c(0x07D0), 1e-6);
    MU_ASSERT_NEAR(85.0, ds18b20_raw_to_c(0x0550), 1e-6);
    MU_ASSERT_NEAR(25.0625, ds18b20_raw_to_c(0x0191), 1e-6);
    MU_ASSERT_NEAR(10.125, ds18b20_raw_to_c(0x00A2), 1e-6);
    MU_ASSERT_NEAR(0.5, ds18b20_raw_to_c(0x0008), 1e-6);
    MU_ASSERT_NEAR(0.0, ds18b20_raw_to_c(0x0000), 1e-6);
    MU_ASSERT_NEAR(-0.5, ds18b20_raw_to_c((int16_t)0xFFF8), 1e-6);
    MU_ASSERT_NEAR(-10.125, ds18b20_raw_to_c((int16_t)0xFF5E), 1e-6);
    MU_ASSERT_NEAR(-25.0625, ds18b20_raw_to_c((int16_t)0xFE6F), 1e-6);
    MU_ASSERT_NEAR(-55.0, ds18b20_raw_to_c((int16_t)0xFC90), 1e-6);
}

static sens_status_t run_cycle(ds18b20_t *s, uint32_t max_ms)
{
    for (uint32_t i = 0; i <= max_ms; i++) {
        sens_status_t st = ds18b20_update(s, fake_time_get());
        if (st != SENS_ERR_NOT_READY) return st;
        fake_time_advance(1);
    }
    return SENS_ERR_TIMEOUT;
}

static void test_ds18b20_presence_detection(void)
{
    fake_ds18b20_t f; ds18b20_t s;
    fake_ds18b20_setup(&f, 21.5f);
    MU_ASSERT_STATUS(SENS_OK, ds18b20_init(&s));
    f.present = false;
    MU_ASSERT_STATUS(SENS_ERR_BUS, ds18b20_init(&s));
    MU_ASSERT_STATUS(SENS_ERR_PARAM, ds18b20_init(NULL));
}

static void test_ds18b20_full_conversion_cycle_and_timing(void)
{
    fake_ds18b20_t f; ds18b20_t s;
    fake_ds18b20_setup(&f, 21.5f);
    ds18b20_init(&s);
    uint32_t t0 = fake_time_get();
    MU_ASSERT_STATUS(SENS_ERR_NOT_READY, ds18b20_update(&s, t0));            /* lance la conversion */
    fake_time_advance(700);
    MU_ASSERT_STATUS(SENS_ERR_NOT_READY, ds18b20_update(&s, fake_time_get())); /* 750 ms pas écoulées */
    fake_time_advance(100);
    MU_ASSERT_STATUS(SENS_OK, ds18b20_update(&s, fake_time_get()));
    MU_ASSERT_NEAR(21.5, s.temp_c, 0.001);
    MU_ASSERT(s.valid);
    MU_ASSERT_INT(1, f.conversions);
    f.raw = (int16_t)0x0191;                                                  /* la température change */
    MU_ASSERT_STATUS(SENS_OK, run_cycle(&s, 2000));
    MU_ASSERT_NEAR(25.0625, s.temp_c, 0.001);
    f.raw = (int16_t)0xFF5E;                                                  /* température négative */
    MU_ASSERT_STATUS(SENS_OK, run_cycle(&s, 2000));
    MU_ASSERT_NEAR(-10.125, s.temp_c, 0.001);
}

static void test_ds18b20_crc_error_is_flagged_then_recovers(void)
{
    fake_ds18b20_t f; ds18b20_t s;
    fake_ds18b20_setup(&f, 21.5f);
    ds18b20_init(&s);
    run_cycle(&s, 2000);
    f.corrupt_crc = true;
    MU_ASSERT_STATUS(SENS_ERR_CRC, run_cycle(&s, 2000));
    MU_ASSERT(!s.valid);
    MU_ASSERT(s.error_count >= 1);
    f.corrupt_crc = false;
    MU_ASSERT_STATUS(SENS_OK, run_cycle(&s, 2000));
    MU_ASSERT(s.valid);
}

static void test_ds18b20_probe_unplugged_midway(void)
{
    fake_ds18b20_t f; ds18b20_t s;
    fake_ds18b20_setup(&f, 21.5f);
    ds18b20_init(&s);
    run_cycle(&s, 2000);
    f.present = false;
    MU_ASSERT_STATUS(SENS_ERR_BUS, run_cycle(&s, 2000));
    MU_ASSERT(!s.valid);
    f.present = true;
    MU_ASSERT_STATUS(SENS_OK, run_cycle(&s, 2000));
}

static void test_ds18b20_power_on_value_85c_is_ignored_only_at_first(void)
{
    fake_ds18b20_t f; ds18b20_t s;
    fake_ds18b20_setup(&f, 85.0f);                                            /* la sonde renvoie 85,0 (valeur de démarrage) */
    ds18b20_init(&s);
    int ok = 0;
    for (int i = 0; i < 2500; i++) {                                           /* plusieurs cycles : jamais de valeur publiée */
        if (ds18b20_update(&s, fake_time_get()) == SENS_OK) ok++;
        fake_time_advance(1);
    }
    MU_ASSERT_INT(0, ok);
    MU_ASSERT(!s.valid);
    MU_ASSERT(f.conversions >= 2);                                             /* la sonde continue d'être interrogée */
    f.raw = (int16_t)lroundf(30.0f * 16.0f);
    MU_ASSERT_STATUS(SENS_OK, run_cycle(&s, 2000));
    f.raw = 0x0550;                                                            /* maintenant 85 °C est une vraie mesure */
    MU_ASSERT_STATUS(SENS_OK, run_cycle(&s, 2000));
    MU_ASSERT_NEAR(85.0, s.temp_c, 0.001);
}

static void test_ds18b20_out_of_range_value_rejected(void)
{
    fake_ds18b20_t f; ds18b20_t s;
    fake_ds18b20_setup(&f, 21.5f);
    ds18b20_init(&s);
    run_cycle(&s, 2000);
    f.raw = 0x7FF0;                                                            /* 2047 °C : impossible */
    MU_ASSERT_STATUS(SENS_ERR_RANGE, run_cycle(&s, 2000));
    MU_ASSERT(!s.valid);
}

static void test_ds18b20_unplugged_between_conversion_and_readout(void)
{
    fake_ds18b20_t f; ds18b20_t s;
    fake_ds18b20_setup(&f, 21.5f);
    ds18b20_init(&s);
    MU_ASSERT_STATUS(SENS_ERR_NOT_READY, ds18b20_update(&s, fake_time_get()));   /* conversion lancée */
    f.present = false;                                                            /* débranchée pendant les 750 ms */
    fake_time_advance(800);
    MU_ASSERT_STATUS(SENS_ERR_BUS, ds18b20_update(&s, fake_time_get()));
    MU_ASSERT(!s.valid);
    f.present = true;
    MU_ASSERT_STATUS(SENS_OK, run_cycle(&s, 2000));
}

int main(void)
{
    MU_RUN(test_ina_init_checks_ids_and_writes_config);
    MU_RUN(test_ina_init_errors);
    MU_RUN(test_ina_read_voltage_current_power);
    MU_RUN(test_ina_negative_current_and_other_shunt_value);
    MU_RUN(test_ina_saturation_flag_and_bus_msb_masked);
    MU_RUN(test_ina_read_bus_error);
    MU_RUN(test_ds18b20_crc8_known_vectors);
    MU_RUN(test_ds18b20_raw_conversion_datasheet_values);
    MU_RUN(test_ds18b20_presence_detection);
    MU_RUN(test_ds18b20_full_conversion_cycle_and_timing);
    MU_RUN(test_ds18b20_crc_error_is_flagged_then_recovers);
    MU_RUN(test_ds18b20_probe_unplugged_midway);
    MU_RUN(test_ds18b20_power_on_value_85c_is_ignored_only_at_first);
    MU_RUN(test_ds18b20_out_of_range_value_rejected);
    MU_RUN(test_ds18b20_unplugged_between_conversion_and_readout);
    return MU_SUMMARY();
}
