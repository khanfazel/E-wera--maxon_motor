#include <Definitions.h>
#include <iostream>
#include <cstdint>
#include <cstring>

static bool ok(int result, const char* what)
{
    if(result == 0)
    {
        std::cerr << "[ERROR] " << what << " failed\n";
        return false;
    }
    return true;
}

static void printErrorInfo(unsigned int errCode)
{
    char errText[256] = {};
    VCS_GetErrorInfo(errCode, errText, static_cast<unsigned short>(sizeof(errText)));
    std::cerr << "[EPOS ERROR] " << errText << "\n";
}

static bool readObject(
    void* handle,
    uint16_t nodeId,
    uint16_t index,
    uint8_t subIndex,
    void* data,
    unsigned int size,
    const char* name)
{
    unsigned int err = 0;
    unsigned int bytesRead = 0;

    if(!ok(VCS_GetObject(handle, nodeId, index, subIndex, data, size, &bytesRead, &err), name))
    {
        printErrorInfo(err);
        return false;
    }

    if(bytesRead != size)
    {
        std::cerr << "[ERROR] " << name << ": wrong byte count = " << bytesRead
                  << ", expected = " << size << "\n";
        return false;
    }

    return true;
}

int main()
{
    unsigned int err = 0;

    const char* deviceName    = "EPOS4";
    const char* protocolStack = "MAXON SERIAL V2";
    const char* interfaceName = "USB";
    const char* portName      = "USB0";
    uint16_t nodeId = 3;   // change if needed

    void* handle = VCS_OpenDevice(
        const_cast<char*>(deviceName),
        const_cast<char*>(protocolStack),
        const_cast<char*>(interfaceName),
        const_cast<char*>(portName),
        &err);

    if(handle == nullptr)
    {
        std::cerr << "[ERROR] VCS_OpenDevice failed\n";
        printErrorInfo(err);
        return 1;
    }

    int16_t ratedTorque_mNm = 0;   // 0x6076
    int32_t actualPosition  = 0;   // 0x6064
    int16_t actualTorque    = 0;   // 0x6077

    bool ok1 = readObject(handle, nodeId, 0x6076, 0x00,
                          &ratedTorque_mNm, sizeof(ratedTorque_mNm),
                          "Read 0x6076 Motor rated torque");

    bool ok2 = readObject(handle, nodeId, 0x6064, 0x00,
                          &actualPosition, sizeof(actualPosition),
                          "Read 0x6064 Position actual value");

    bool ok3 = readObject(handle, nodeId, 0x6077, 0x00,
                          &actualTorque, sizeof(actualTorque),
                          "Read 0x6077 Torque actual value");

    std::cout << "\n===== EPOS READOUT =====\n";

    if(ok1)
        std::cout << "Motor rated torque (0x6076) = " << ratedTorque_mNm << " mNm\n";

    if(ok2)
        std::cout << "Actual position   (0x6064) = " << actualPosition << " counts\n";

    if(ok3)
        std::cout << "Actual torque     (0x6077) = " << actualTorque
                  << " permille of rated torque\n";

    VCS_CloseDevice(handle, &err);
    return 0;
}
