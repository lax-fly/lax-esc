#include "crc.h"
#include "at32f421_crc.h"

static Crc crc;
static bool crc_inited = false;

void Crc::set_start(uint8_t val)
{
    CRC->idt = val;
    CRC->ctrl_bit.rst = 1;
}

void Crc::set_poly(uint8_t poly)
{
    CRC->poly = poly;
}

uint8_t Crc::calc(uint8_t *data, uint32_t sz)
{
    CRC->ctrl_bit.rst = 1;
    for (uint32_t i = 0; i < sz; i++)
    {   // careful, the AT's crc length is determined by the length you write, so there must be a (uint8_t *) cast to have crc8 
        (*(uint8_t *)&CRC->dt) = data[i];
    }

    return (CRC->dt);
}

uint32_t Crc::calc(uint8_t data)
{
    // careful, the AT's crc length is determined by the length you write, so there must be a (uint8_t *) cast to have crc8 
    (*(uint8_t *)&CRC->dt) = data;
    return (CRC->dt);
}

CrcIf *CrcIf::singleton()
{
    if (crc_inited)
        return &crc;
    crc_inited = true;
    crm_periph_clock_enable(CRM_CRC_PERIPH_CLOCK, TRUE);
    crc_init_data_set(0x00000000);
    crc_poly_size_set(CRC_POLY_SIZE_8B);
    crc_poly_value_set(0xB7);
    crc_reverse_input_data_set(CRC_REVERSE_INPUT_NO_AFFECTE);
    crc_reverse_output_data_set(CRC_REVERSE_OUTPUT_NO_AFFECTE);
    crc_data_reset();
    return &crc;
}
