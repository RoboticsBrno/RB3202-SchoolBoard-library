#pragma once

#include <stdint.h>
#include <vector>
#include <esp_err.h>
#include <esp_log.h>

namespace pixy2 {

static constexpr const uint8_t HDR0_CSUM = 0xAF;
static constexpr const uint8_t HDR0_PLAIN = 0xAE;
static constexpr const uint8_t HDR1 = 0xC1;

template<typename T> class Pixy2;
class Pixy2_I2C;

enum PacketType : uint8_t {
    ERROR = 0x03,
    GET_VERSION = 0x0E,
    GET_VERSION_RESPONSE = 0x0F,
    GET_BLOCKS = 0x20,
    GET_BLOCKS_RESPONSE = 0x21,
};

template<size_t N>
class PacketRequest {
    template<typename T> friend class Pixy2;
    friend class Pixy2_I2C;
public:
    ~PacketRequest() {}

private:
    template<typename T>
    PacketRequest(PacketType type, T const (&bytes)[N]) {
        m_raw[0] = HDR0_PLAIN;
        m_raw[1] = HDR1;
        m_raw[2] = type;
        m_raw[3] = N;
        for(size_t i = 0; i < N; ++i) {
            m_raw[4 + i] = bytes[i];
        }
    }

    PacketRequest(PacketType type) {
        m_raw[0] = HDR0_PLAIN;
        m_raw[1] = HDR1;
        m_raw[2] = type;
        m_raw[3] = 0;
    }

    static constexpr size_t rawSize() { return 4 + N; }

    uint8_t m_raw[rawSize()];
};

class PacketResponse {
    template<typename T> friend class Pixy2;
    friend class Pixy2_I2C;
public:
    PacketResponse() {}
    ~PacketResponse() {}

    uint8_t headerSize() const {
        return m_raw[0] == HDR0_CSUM ? 6 : 4;
    }

    uint8_t dataLen() const {
        return m_raw[3];
    }

    PacketType type() const {
        return PacketType(m_raw[2]);
    }

    template<typename T>
    T get(uint8_t idx) {
        size_t end = headerSize() + idx + sizeof(T);
        if(end > m_raw.size()) {
            ESP_LOGE("pixy2", "attempted to read until %d, but only have %d bytes.", end, m_raw.size());
            return T();
        }
        return *((T*)(m_raw.data() + headerSize() + idx));
    }

    uint32_t get(uint8_t idx) {
        size_t end = headerSize() + idx + sizeof(uint32_t);
        if(end > m_raw.size()) {
            ESP_LOGE("pixy2", "attempted to read until %d, but only have %d bytes.", end, m_raw.size());
            return uint32_t(0);
        }

        uint8_t *buffer = (m_raw.data() + headerSize() + idx);

        return *( (uint32_t*) buffer );
    }

private:
    esp_err_t checkCsum() const {
        if(m_raw[0] != HDR0_CSUM) {
            return ESP_OK;
        }

        const uint16_t expectedCsum = m_raw[4] | (m_raw[5] << 8);
        uint16_t calculatedCsum = 0;
        for(size_t i = 6; i < m_raw.size(); ++i) {
            calculatedCsum += m_raw[i];
        }

        if (expectedCsum != calculatedCsum)
        {
            ESP_LOGE("pixy2", "checksums don't match: %04x != %04x", calculatedCsum, expectedCsum);
            return ESP_ERR_INVALID_CRC;
        }
        return ESP_OK;
    }

    std::vector<uint8_t> m_raw;
};

struct ColorBlock {
    uint16_t signature;
    uint16_t x, y, w, h;
    int16_t angle;
    uint8_t index;
    uint8_t age;
} __attribute__((packed));

struct VersionResponse {
    uint16_t hw_version;
    uint8_t fw_version_major;
    uint8_t fw_version_minor;
    uint16_t fw_build_number;
    char fw_type[10];
} __attribute__((packed));

};
