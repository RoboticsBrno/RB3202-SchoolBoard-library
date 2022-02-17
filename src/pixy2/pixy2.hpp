#pragma once

#include <driver/spi_master.h>
#include <mutex>
#include <freertos/task.h>
#include <tuple>

#include "packet.hpp"

namespace pixy2 {

struct GetBlocksContext {
    std::vector<ColorBlock> blocks;
    PacketResponse resp;
};

template<typename LinkType>
class Pixy2 {
public:
    static constexpr const esp_err_t ERR_PIXY_BUSY = 0x10;

    Pixy2(LinkType&& link): m_link(std::move(link)) {

    }

    Pixy2(Pixy2&& other): m_link(std::move(other.m_link)) {

    }
    ~Pixy2() { }

    esp_err_t waitForStartup(VersionResponse *captureVersion = nullptr, TickType_t timeout = pdMS_TO_TICKS(5000)) const;
    esp_err_t getVersion(VersionResponse& dest) const;
    esp_err_t getBlocks(uint8_t signaturesMask, uint8_t maxBlocks, GetBlocksContext& ctx) const;

    template<typename T, size_t N>
    static PacketRequest<N> request(PacketType type, T const (&bytes)[N]) {
        return PacketRequest<N>(type, bytes);
    }

    static PacketRequest<0> request(PacketType type) {
        return PacketRequest<0>(type);
    }

    template<size_t N>
    esp_err_t transact(const PacketRequest<N>& request, PacketResponse& response) const {
        return transact(request.m_raw, request.rawSize(), response);
    }

private:
    Pixy2(const Pixy2&) = delete;

    esp_err_t transact(const uint8_t *reqData, size_t reqLen, PacketResponse& response) const;

    esp_err_t waitForSyncLocked(PacketResponse& resp, uint16_t attempts = 32) const;
    esp_err_t receivePacketLocked(PacketResponse& resp) const;

    mutable std::mutex m_linkMutex;
    LinkType m_link;
};

template<typename LinkType>
esp_err_t Pixy2<LinkType>::waitForSyncLocked(PacketResponse& resp, uint16_t attempts) const {
    uint8_t current, prev = 0;
    for (size_t i = 0; i < attempts; ++i)
    {
        auto err = m_link.receiveData(&current, 1);
        if (err != ESP_OK)
        {
            return err;
        }

        if (current == HDR1 && (prev == HDR0_PLAIN || prev == HDR0_CSUM))
        {
            resp.m_raw = { prev, current };
            return ESP_OK;
        }
        prev = current;
    }
    return ESP_ERR_TIMEOUT;
}

template<typename LinkType>
esp_err_t Pixy2<LinkType>::receivePacketLocked(PacketResponse& resp) const {
    auto err = waitForSyncLocked(resp);
    if (err != ESP_OK)
    {
        return err;
    }

    auto& raw = resp.m_raw;
    const auto hdrSize = resp.headerSize();

    raw.resize(hdrSize);
    err = m_link.receiveData(raw.data() + 2,  hdrSize - 2);
    if (err != ESP_OK)
    {
        return err;
    }

    if (resp.dataLen() > 0)
    {
        raw.resize(hdrSize + resp.dataLen());
        err = m_link.receiveData(raw.data() + hdrSize, resp.dataLen());
        if (err != ESP_OK)
        {
            return err;
        }
    }

    return resp.checkCsum();
}

template<typename LinkType>
esp_err_t Pixy2<LinkType>::transact(const uint8_t *reqData, size_t reqLen, PacketResponse& response) const {
    std::lock_guard<std::mutex> l(m_linkMutex);

    auto err = m_link.sendData(reqData, reqLen);
    if(err != ESP_OK) {
        return err;
    }

    return receivePacketLocked(response);
}

template<typename LinkType>
esp_err_t Pixy2<LinkType>::waitForStartup(VersionResponse *captureVersion, TickType_t timeout) const {
    VersionResponse version;
    const auto start = xTaskGetTickCount();
    const auto timeoutAt = start + timeout;
    while (xTaskGetTickCount() < timeoutAt) {
        auto err = getVersion(version);
        if(err == ESP_OK) {
            if(captureVersion) {
                *captureVersion = version;
            }
            return ESP_OK;
        } else if(err != ESP_ERR_TIMEOUT) {
            return err;
        }
        vTaskDelay(1);
    }
    return ESP_ERR_TIMEOUT;
}

template<typename LinkType>
esp_err_t Pixy2<LinkType>::getVersion(VersionResponse& dest) const {
    const auto verReq = Pixy2::request(PacketType::GET_VERSION);

    PacketResponse resp;
    auto err = transact(verReq, resp);
    if(err != ESP_OK) {
        return err;
    }

    if(resp.type() != PacketType::GET_VERSION_RESPONSE) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    dest = resp.get<VersionResponse>(0);
    return ESP_OK;
}

template<typename LinkType>
esp_err_t Pixy2<LinkType>::getBlocks(uint8_t signaturesMask, uint8_t maxBlocks, GetBlocksContext& ctx) const {
    const auto blocksReq = Pixy2::request(PacketType::GET_BLOCKS, { signaturesMask, maxBlocks });

    auto& r = ctx.resp;

    auto err = transact(blocksReq, r);
    if(err != ESP_OK) {
        return err;
    }

    if(r.type() == PacketType::ERROR) {
        return ERR_PIXY_BUSY;
    } else if(r.type() != PacketType::GET_BLOCKS_RESPONSE) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    ctx.blocks.resize(r.dataLen() / sizeof(ColorBlock));
    size_t off = 0;
    for(auto& b : ctx.blocks) {
        b = r.get<ColorBlock>(off);
        off += sizeof(ColorBlock);
    }
    return ESP_OK;
}

};
