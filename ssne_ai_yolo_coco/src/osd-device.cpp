/*
 * @Author: Jingwen Bai
 * @Date: 2024-07-04 11:07:00
 * @Description: osd device
 * @Filename: osd-device.cpp
 */

#include <iostream>
#include <fstream>
#include <cstring>
#include <cerrno>
#include <sys/stat.h>
#include <unistd.h>

#include "../include/osd-device.hpp"
#include "log.hpp"
using namespace fdevice;
namespace sst{
namespace device{
namespace osd{

OsdDevice::OsdDevice()
    :m_height(0),
    m_width(0){

}

OsdDevice::~OsdDevice(){
}

void OsdDevice::Initialize(int width, int height, const char* bitmap_lut_path){
    m_width = width;
    m_height = height;

    // 加载颜色查找表
    if (bitmap_lut_path != nullptr && strlen(bitmap_lut_path) > 0) {
        if (LoadLutFile(bitmap_lut_path) != 0) {
            std::cerr << "[OsdDevice] Warning: Failed to load bitmap LUT, using default LUT" << std::endl;
            LoadLutFile(m_osd_lut_path.c_str());
        }
    } else {
        LoadLutFile(m_osd_lut_path.c_str());
    }

    m_osd_handle = osd_open_device();
    if (m_osd_handle == INVALID_HANDLE) {
        std::cerr << "[OsdDevice][INIT] osd_open_device failed" << std::endl;
        return;
    }
    // osd_init_device 必须在创建图层前调用
    int ret = osd_init_device(m_osd_handle, OSD_LAYER_SIZE, (char*)m_pcolor_lut);
    if (ret != 0) {
        std::cerr << "[OsdDevice][INIT] osd_init_device failed, ret=" << ret
                  << ", layers=" << OSD_LAYER_SIZE << std::endl;
        return;
    }

    // Layer 0/1: 矩形检测框 + 危险区域框
    int dma_size = 0x8000;  // 32KB
    for(int layer_index = 0; layer_index < 2; layer_index++){
        ret = osd_alloc_buffer(m_osd_handle, m_layer_dma[layer_index].dma, dma_size);
        if (ret != 0) {
            std::cerr << "[OsdDevice][INIT] primary DMA allocation failed, layer="
                      << layer_index << ", ret=" << ret << std::endl;
            continue;
        }
        ret = osd_alloc_buffer(m_osd_handle, m_layer_dma[layer_index].dma_2, dma_size);
        if (ret != 0) {
            std::cerr << "[OsdDevice][INIT] secondary DMA allocation failed; using single buffer, layer="
                      << layer_index << ", ret=" << ret << std::endl;
            m_layer_dma[layer_index].dma_2 = nullptr;
        }
        int dma_fd = osd_get_buffer_fd(m_osd_handle, m_layer_dma[layer_index].dma);
        if (dma_fd < 0) {
            std::cerr << "[OsdDevice][INIT] cannot get DMA fd, layer=" << layer_index
                      << ", ret=" << dma_fd << std::endl;
            continue;
        }

        LAYER_ATTR_S osd_layer = {};
        osd_layer.codeTYPE = SS_TYPE_QUADRANGLE;
        osd_layer.layer_data_QR.osd_buf.buf_type = BUFFER_TYPE_DMABUF;
        osd_layer.layer_data_QR.osd_buf.buf.fd_dmabuf = dma_fd;
        osd_layer.layerStart.layer_start_x = 0;
        osd_layer.layerStart.layer_start_y = 0;
        osd_layer.layerSize.layer_width = m_width;
        osd_layer.layerSize.layer_height = m_height;
        osd_layer.layer_rgn = {TYPE_GRAPHIC, {m_width, m_height}};
        ret = osd_create_layer(m_osd_handle, (ssLAYER_HANDLE)layer_index, &osd_layer);
        if (ret != 0) {
            std::cerr << "[OsdDevice][INIT] graphic layer creation failed, layer="
                      << layer_index << ", ret=" << ret << std::endl;
            continue;
        }
        ret = osd_set_layer_buffer(m_osd_handle, (ssLAYER_HANDLE)layer_index,
                                   m_layer_dma[layer_index]);
        if (ret != 0) {
            std::cerr << "[OsdDevice][INIT] graphic layer buffer setup failed, layer="
                      << layer_index << ", ret=" << ret << std::endl;
        }
    }

    // Layer 2/3: 多边形危险区域位图 + ALERT位图 (RLE编码)。
    // 多边形层先创建，确保位图资源不足时危险区轮廓优先于ALERT图标。
    for (int layer_index = 2; layer_index < OSD_LAYER_SIZE; ++layer_index) {
        // 多边形可能覆盖较大画面，预留更大的RLE编码缓冲区。
        const int dma_size = (layer_index == 2) ? 0x40000 : 0x20000;
        ret = osd_alloc_buffer(m_osd_handle, m_layer_dma[layer_index].dma, dma_size);
        if (ret != 0) {
            std::cerr << "[OsdDevice][INIT] primary DMA allocation failed, layer="
                      << layer_index << ", ret=" << ret << std::endl;
            continue;
        }
        ret = osd_alloc_buffer(m_osd_handle, m_layer_dma[layer_index].dma_2, dma_size);
        if (ret != 0) {
            std::cerr << "[OsdDevice][INIT] secondary DMA allocation failed; using single buffer, layer="
                      << layer_index << ", ret=" << ret << std::endl;
            m_layer_dma[layer_index].dma_2 = nullptr;
        }
        int dma_fd = osd_get_buffer_fd(m_osd_handle, m_layer_dma[layer_index].dma);
        if (dma_fd < 0) {
            std::cerr << "[OsdDevice][INIT] cannot get DMA fd, layer=" << layer_index
                      << ", ret=" << dma_fd << std::endl;
            continue;
        }

        LAYER_ATTR_S osd_layer = {};
        osd_layer.codeTYPE = SS_TYPE_RLE;
        osd_layer.layer_data_RLE.osd_buf.buf_type = BUFFER_TYPE_DMABUF;
        osd_layer.layer_data_RLE.osd_buf.buf.fd_dmabuf = dma_fd;
        osd_layer.layerStart.layer_start_x = 0;
        osd_layer.layerStart.layer_start_y = 0;
        osd_layer.layerSize.layer_width = m_width;
        osd_layer.layerSize.layer_height = m_height;
        osd_layer.layer_rgn = {TYPE_IMAGE, {m_width, m_height}};
        ret = osd_create_layer(m_osd_handle, (ssLAYER_HANDLE)layer_index, &osd_layer);
        if (ret != 0) {
            std::cerr << "[OsdDevice][INIT] bitmap layer creation failed, layer="
                      << layer_index << ", ret=" << ret << std::endl;
            continue;
        }
        ret = osd_set_layer_buffer(m_osd_handle, (ssLAYER_HANDLE)layer_index,
                                   m_layer_dma[layer_index]);
        if (ret != 0) {
            std::cerr << "[OsdDevice][INIT] bitmap layer buffer setup failed, layer="
                      << layer_index << ", ret=" << ret << std::endl;
        }
    }
}

void OsdDevice::ClearLayer(int layer_id){
    if (layer_id < 0 || layer_id >= OSD_LAYER_SIZE) return;
    osd_clean_layer(m_osd_handle, (ssLAYER_HANDLE)layer_id);
}


void OsdDevice::Release(){

    for(int i = 0; i < OSD_LAYER_SIZE; i++){
        osd_destroy_layer(m_osd_handle, (ssLAYER_HANDLE)i);

        if(m_layer_dma[i].dma != nullptr)
            osd_delete_buffer(m_osd_handle, m_layer_dma[i].dma);
        if(m_layer_dma[i].dma_2 != nullptr)
            osd_delete_buffer(m_osd_handle, m_layer_dma[i].dma_2);
    }

    if(m_pcolor_lut != nullptr){
        delete[] m_pcolor_lut;
        m_pcolor_lut = nullptr;
    }

    osd_close_device(m_osd_handle);
}



int OsdDevice::LoadLutFile(const char* filename){
    struct stat file_stat;
    if (stat(filename, &file_stat) != 0) {
        std::cerr << "[OsdDevice] ERROR: File does not exist or cannot access: " << filename << std::endl;
        std::cerr << "[OsdDevice] Error code: " << errno << " (" << strerror(errno) << ")" << std::endl;
        return -1;
    }

    if (file_stat.st_size <= 0) {
        std::cerr << "[OsdDevice] ERROR: Invalid file size: " << file_stat.st_size << " bytes" << std::endl;
        return -1;
    }

    if (access(filename, R_OK) != 0) {
        std::cerr << "[OsdDevice] ERROR: No read permission for file: " << filename << std::endl;
        std::cerr << "[OsdDevice] Error code: " << errno << " (" << strerror(errno) << ")" << std::endl;
        return -1;
    }

    std::ifstream file(filename, std::ios::binary | std::ios::in | std::ios::ate);
    if (!file) {
        std::cerr << "[OsdDevice] ERROR: Cannot open file: " << filename << std::endl;
        std::cerr << "[OsdDevice] Error code: " << errno << " (" << strerror(errno) << ")" << std::endl;
        return -1;
    }

    m_file_size = file.tellg();
    if (m_file_size <= 0) {
        std::cerr << "[OsdDevice] ERROR: Invalid file size from stream: " << m_file_size << " bytes" << std::endl;
        file.close();
        return -1;
    }

    m_pcolor_lut = new uint8_t[m_file_size];
    file.seekg(0, std::ios::beg);
    file.read((char*)m_pcolor_lut, m_file_size);

    if (file.gcount() != m_file_size) {
        std::cerr << "[OsdDevice] ERROR: Failed to read complete file. Expected: " << m_file_size
                  << " bytes, Read: " << file.gcount() << " bytes" << std::endl;
        delete[] m_pcolor_lut;
        m_pcolor_lut = nullptr;
        file.close();
        return -1;
    }

    file.close();
    return 0;
}

void OsdDevice::Draw(std::vector<OsdQuadRangle> &quad_rangle){
    if ((quad_rangle.size() == 0)){
        osd_clean_all_layer(m_osd_handle);
        return;
    }

    for(auto &q : quad_rangle){
        GenQrangleBox(q.box, q.border);
        COVER_ATTR_S qrangle_attr = {q.color, q.type, q.alpha, m_qrangle_out, m_qrangle_in};
        osd_add_quad_rangle(m_osd_handle, &qrangle_attr);
    }

    osd_flush_quad_rangle(m_osd_handle);
}

void OsdDevice::Draw(std::vector<OsdQuadRangle> &quad_rangle, int layer_id){
    if ((quad_rangle.size() == 0)){
        osd_clean_layer(m_osd_handle, (ssLAYER_HANDLE)layer_id);
        LOG_DEBUG("Draw --- osd_clean_layer\n");
        return;
    }
    int ret = 0;

    for(auto &q : quad_rangle){
        LOG_DEBUG("Draw --- q.box: %f, %f, %f, %f\n", q.box[0], q.box[1], q.box[2], q.box[3]);
        GenQrangleBox(q.box, q.border);
        COVER_ATTR_S qrangle_attr = {q.color, q.type, q.alpha, m_qrangle_out, m_qrangle_in};
        ret = osd_add_quad_rangle_layer(m_osd_handle, (ssLAYER_HANDLE)layer_id, &qrangle_attr);
        LOG_DEBUG("Draw --- osd_add_quad_rangle_layer ret: %d\n", ret);
    }

    osd_flush_quad_rangle_layer(m_osd_handle, (ssLAYER_HANDLE)layer_id);
}

void OsdDevice::Draw(std::vector<std::array<float, 4>>& boxes, int border, int layer_id, tagQUADRANGLETYPE type, tagALPHATYPE alpha, int color){
    if ((boxes.size() == 0)){
        osd_clean_layer(m_osd_handle, (ssLAYER_HANDLE)layer_id);
        return;
    }

    osd_clean_layer(m_osd_handle, (ssLAYER_HANDLE)layer_id);
    int ret = 0;
    for (auto &box : boxes){
        GenQrangleBox(box, border);
        COVER_ATTR_S qrangle_attr = {color, type, alpha, m_qrangle_out, m_qrangle_in};
        ret = osd_add_quad_rangle_layer(m_osd_handle, (ssLAYER_HANDLE)layer_id, &qrangle_attr);
    }

    osd_flush_quad_rangle_layer(m_osd_handle, (ssLAYER_HANDLE)layer_id);
}


// LUT在Initialize时已加载，这里只操作位图绘制
bool OsdDevice::DrawTexture(const char* bitmap_path, const char* lut_path, int layer_id, int pos_x, int pos_y, fdevice::ALPHATYPE alpha) {
    (void)lut_path;
    if (bitmap_path == nullptr || layer_id < 0 || layer_id >= OSD_LAYER_SIZE) {
        return false;
    }

    fdevice::BITMAP_INFO_S bm_info = {};
    bm_info.pSSbmpFile = bitmap_path;
    bm_info.alpha = alpha;
    bm_info.position.x = pos_x;
    bm_info.position.y = pos_y;

    LOG_DEBUG("[OsdDevice] Drawing texture: %s", bitmap_path);
    LOG_DEBUG(" at position %d,%d", pos_x, pos_y);
    LOG_DEBUG(" layer_id=%d\n", layer_id);

    int ret = osd_add_texture_layer(m_osd_handle, (ssLAYER_HANDLE)layer_id, &bm_info);
    if (ret != 0) {
        std::cerr << "[OsdDevice] ERROR: osd_add_texture_layer failed! ret=" << ret
                  << ", layer_id=" << layer_id << std::endl;
        if (ret == -1) {
            std::cerr << "[OsdDevice] Layer does not exist or type mismatch (should be TYPE_IMAGE)" << std::endl;
        } else if (ret == -2) {
            std::cerr << "[OsdDevice] Bitmap add failed (encoding data too large or invalid file)" << std::endl;
        }
        return false;
    }
    LOG_DEBUG("[OsdDevice] osd_add_texture_layer succeeded\n");

    ret = osd_flush_texture_layer(m_osd_handle, (ssLAYER_HANDLE)layer_id);
    if (ret != 0) {
        std::cerr << "[OsdDevice] ERROR: osd_flush_texture_layer failed! ret=" << ret
                  << ", layer_id=" << layer_id << std::endl;
    } else {
        LOG_DEBUG("[OsdDevice] Texture drawn successfully\n");
    }
    return ret == 0;
}

void OsdDevice::GenQrangleBox(std::array<float, 4>& det, int border){
    std::array<int, 16> box;

    box[0] = std::min(m_width, std::max(0, int(det[0]+border)));
    box[1] = std::min(m_height, std::max(0, int(det[1]+border)));
    box[2] = std::min(m_width, std::max(0, int(det[0]+border)));
    box[3] = std::min(m_height, std::max(0, int(det[3]-border)));
    box[4] = std::min(m_width, std::max(0, int(det[2]-border)));
    box[5] = std::min(m_height, std::max(0, int(det[3]-border)));
    box[6] = std::min(m_width, std::max(0, int(det[2]-border)));
    box[7] = std::min(m_height, std::max(0, int(det[1]+border)));

    box[8] = std::min(m_width, std::max(0, int(det[0]-border)));
    box[9] = std::min(m_height, std::max(0, int(det[1]-border)));
    box[10] = std::min(m_width, std::max(0, int(det[0]-border)));
    box[11] = std::min(m_height, std::max(0, int(det[3]+border)));
    box[12] = std::min(m_width, std::max(0, int(det[2]+border)));
    box[13] = std::min(m_height, std::max(0, int(det[3]+border)));
    box[14] = std::min(m_width, std::max(0, int(det[2]+border)));
    box[15] = std::min(m_height, std::max(0, int(det[1]-border)));

    m_qrangle_in.points[0]={box[0], box[1]};
    m_qrangle_in.points[1]={box[2], box[3]};
    m_qrangle_in.points[2]={box[4], box[5]};
    m_qrangle_in.points[3]={box[6], box[7]};
    m_qrangle_out.points[0] = {box[8], box[9]};
    m_qrangle_out.points[1] = {box[10], box[11]};
    m_qrangle_out.points[2] = {box[12], box[13]};
    m_qrangle_out.points[3] = {box[14], box[15]};
}

} // namespace osd
} // namespace device
} // namespace sst
