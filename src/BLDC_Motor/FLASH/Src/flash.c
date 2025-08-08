#include "flash.h"
#include <string.h>

motor_config_t m_config;

HAL_StatusTypeDef save_config_to_flash(motor_config_t *data) {
    HAL_StatusTypeDef status;
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t SectorError = 0;

    data->valid_SOF = SOF_FLAG;
    data->valid_EOF = EOF_FLAG;

    HAL_FLASH_Unlock();

    EraseInitStruct.TypeErase    = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    EraseInitStruct.Sector       = FLASH_SECTOR_NUM;
    EraseInitStruct.NbSectors    = 1;

    status = HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError);
    if (status != HAL_OK) {
        HAL_FLASH_Lock();
        return status;
    }

    uint32_t address = FLASH_SECTOR_ADDR;
    uint8_t *src = (uint8_t *)data;

    for (uint32_t i = 0; i < sizeof(motor_config_t); i += 4) {
        uint32_t word = *(uint32_t*)(src + i);
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, word);
        if (status != HAL_OK) {
            HAL_FLASH_Lock();
            return status;
        }
        address += 4;
    }

    HAL_FLASH_Lock();
    return HAL_OK;
}

void read_config_from_flash(motor_config_t *data) {
    memcpy(data, (void*)FLASH_SECTOR_ADDR, sizeof(motor_config_t));

    // Validasi SOF dan EOF
    if (data->valid_SOF != SOF_FLAG || data->valid_EOF != EOF_FLAG) {
        // set default
        default_config(data);
    }
}

void default_config(motor_config_t *data) {
    data->id_kp = 0.01f;
    data->id_ki = 12.0f;
    data->id_out_max = 15.0f;
    data->id_e_deadband = 0.0001f;

    data->iq_kp = 0.01f;
    data->iq_ki = 12.0f;
    data->iq_out_max = 15.0f;
    data->iq_e_deadband = 0.0001f;

    data->speed_kp = 0.05f;
    data->speed_ki = 0.7f;
    data->speed_out_max = 10.0f;
    data->speed_e_deadband = 0.01f;

    data->pos_kp = 4.1f;
    data->pos_ki = 0.0f;
    data->pos_kd = 0.21f;
    data->pos_out_max = 3.0f;
    data->pos_e_deadband = 0.1f;

    data->voffset_a = 1.65f;
    data->voffset_b = 1.65f;

    data->encd_offset = 0.0f;
    memset(m_config.encd_error_comp, 0, sizeof(m_config.encd_error_comp));

    data->freq = 10000;
    data->dir = NORMAL_DIR;
    data->gear_ratio = 1.0f;
}