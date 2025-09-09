#ifdef WIN32
#include <SDKDDKVer.h>
#endif

#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <cppfmu_cs.hpp>
#include <filesystem>
#include <iostream>
#include <string>
#include <variant>
#include <zeabuz/common/utilities/string.hpp>

using zeabuz::common::utilities::string::string_to_uint16;

namespace pt = boost::property_tree;
namespace fs = std::filesystem;

using boost::asio::ip::udp;
using std::string;
using std::vector;

class FmuMtDriver : public cppfmu::SlaveInstance
{
   public:
    FmuMtDriver(cppfmu::FMIString fmuResourceLocation) : _resource_location(fmuResourceLocation)
    {
        FmuMtDriver::Reset();
    }

    void SetupExperiment(cppfmu::FMIBoolean toleranceDefined, cppfmu::FMIReal tolerance, cppfmu::FMIReal tStart,
                         cppfmu::FMIBoolean stopTimeDefined, cppfmu::FMIReal tStop) override
    {
        // Parse XML file
        auto resourcePath = fs::path(_resource_location.substr(_resource_location.find("://") + 4));
        auto xml_path(resourcePath.parent_path() / "modelDescription.xml");

        pt::ptree tree;
        pt::read_xml(xml_path.string(), tree);

        for (pt::ptree::value_type& v : tree.get_child("fmiModelDescription.ModelVariables")) {
            if (v.first == "ScalarVariable") {
                auto vr = v.second.get<int>("<xmlattr>.valueReference");
                auto name = v.second.get<std::string>("<xmlattr>.name");

                // std::cout << "Creating Signal: " << name << "\t ValueReference: " << vr << std::endl;

                if (v.second.get_child_optional("Real")) {
                    real_signals[vr] = v.second.get<cppfmu::FMIReal>("Real.<xmlattr>.start", 0.0);
                }
                else if (v.second.get_child_optional("Integer")) {
                    integer_signals[vr] = v.second.get<cppfmu::FMIInteger>("Integer.<xmlattr>.start", 0);
                }
                else if (v.second.get_child_optional("String")) {
                    string_signals[vr] = v.second.get<std::string>("String.<xmlattr>.start", "");
                }
                else if (v.second.get_child_optional("Boolean")) {
                    boolean_signals[vr] = v.second.get<cppfmu::FMIBoolean>("Boolean.<xmlattr>.start", true);
                }

                if (name == "remote_ip") {
                    remote_ip_vr = vr;
                }
                else if (name == "remote_port") {
                    remote_port_vr = vr;
                }
                else if (name == "local_ip") {
                    local_ip_vr = vr;
                }
                else if (name == "local_port") {
                    local_port_vr = vr;
                }
            }
        }
    }

    void ExitInitializationMode() override
    {
        socket = std::make_shared<udp::socket>(io_service);
        socket->open(boost::asio::ip::udp::v4());

        remote_endpoint =
            boost::asio::ip::udp::endpoint(boost::asio::ip::address::from_string(string_signals.at(remote_ip_vr)),
                                           string_to_uint16(string_signals.at(remote_port_vr)));

        local_endpoint =
            boost::asio::ip::udp::endpoint(boost::asio::ip::address::from_string(string_signals.at(local_ip_vr)),
                                           string_to_uint16(string_signals.at(local_port_vr)));

        std::cout << "Binding socket to local endpoint: " << local_endpoint << std::endl;
        socket->bind(local_endpoint);
    }

    void Reset() override
    {
        if (socket && socket->is_open()) {
            std::cout << "Closing socket..." << std::endl;
            socket->close();
        }
    }

    void SetString(const cppfmu::FMIValueReference vr[], std::size_t nvr, const cppfmu::FMIString value[]) override
    {
        std::cout << "Cannot modify server ip and port!" << std::endl;
    }

    void SetReal(const cppfmu::FMIValueReference vr[], std::size_t nvr, const cppfmu::FMIReal value[]) override
    {
        for (std::size_t i = 0; i < nvr; ++i) {
            real_signals.at(vr[i]) = value[i];
        }
    }

    void GetString(const cppfmu::FMIValueReference vr[], std::size_t nvr, cppfmu::FMIString value[]) const override
    {
        for (std::size_t i = 0; i < nvr; ++i) {
            value[i] = string_signals.at(vr[i]).c_str();
        }
    }

    void GetReal(const cppfmu::FMIValueReference vr[], std::size_t nvr, cppfmu::FMIReal value[]) const override
    {
        for (std::size_t i = 0; i < nvr; ++i) {
            value[i] = real_signals.at(vr[i]);
        }
    }

    bool DoStep(cppfmu::FMIReal currentCommunicationPoint, cppfmu::FMIReal /*dt*/, cppfmu::FMIBoolean /*newStep*/,
                cppfmu::FMIReal& /*endOfStep*/) override
    {
        // int vesselID = 197;

        // IO card 37:
        std::stringstream card37;
        card37 << "$MTLLF,0,0,"
               << "37"
               << ","
               << "AI,1,2," << std::fixed << std::setprecision(3) << real_signals.at(3) << ",0"
               << ","
               << "AI,1,3," << std::fixed << std::setprecision(3) << real_signals.at(4) << ",0"
               << ","
               << "DI,1,101," << std::fixed << std::setprecision(0) << real_signals.at(93) << ",1"  // Running
               << ","
               << "DI,1,102," << std::fixed << std::setprecision(0) << real_signals.at(94) << ",1"  // Ready
               << ","
               << "DI,1,121," << std::fixed << std::setprecision(0) << real_signals.at(60) << ",0"  // PowerLim
               << ","
               << "DI,1,1121," << 0 << ",0"  // CustomPanel
               << ","
               << "DI,1,115," << 0 << ",0"  // UPS On Battery
               << ","
               << "DI,1,116," << 0 << ",0"  // UPS Low Battery

               << ","
               << "AI,2,2," << std::fixed << std::setprecision(3) << real_signals.at(5) << ",0"
               << ","
               << "AI,2,3," << std::fixed << std::setprecision(3) << real_signals.at(6) << ",0"
               << ","
               << "DI,2,101," << std::fixed << std::setprecision(0) << real_signals.at(95) << ",1"  // Running
               << ","
               << "DI,2,102," << std::fixed << std::setprecision(0) << real_signals.at(96) << ",1"  // Ready
               << ","
               << "DI,2,121," << 0 << ",1"  // PowerLim

               << ","
               << "DI,0,1165," << 1 << ",101"  // Knapper
               << ","
               << "DI,0,1165," << 1 << ",102"  // Knapper

               << ","
               << "DI,0,2087," << 1 << ",1"  // DP Mode Active
               << ","
               << "DI,0,2088," << 0 << ",1"  // AP Mode Active
               << ","
               << "DI,0,2090," << 0 << ",1"  // AP Mode MMS
               << ","
               << "DI,0,2089," << 1 << ",1"  // DP Mode MMS

               << ","
               << "DI,0,2085," << 1 << ",0"  // Vessel Direction Forward
               << ","
               << "DI,0,2086," << 0 << ",0"  // Vessel Direction Aft
               << ","
               << "DI,0,1138," << std::fixed << std::setprecision(0) << real_signals.at(101) << ",0"  // Nødkjøring

               << "\r\n";

        std::stringstream card38;
        card38 << "$MTLLF,0,0,"
               << "38"
               << ","
               << "AI,3,2," << std::fixed << std::setprecision(3) << real_signals.at(7) << ",0"
               << ","
               << "AI,3,3," << std::fixed << std::setprecision(3) << real_signals.at(8) << ",0"
               << ","
               << "DI,3,101," << std::fixed << std::setprecision(0) << real_signals.at(97) << ",1"  // Running
               << ","
               << "DI,3,102," << std::fixed << std::setprecision(0) << real_signals.at(98) << ",1"  // Ready
               << ","
               << "DI,3,121," << 0 << ",1"  // PowerLim

               << ","
               << "AI,4,2," << std::fixed << std::setprecision(3) << real_signals.at(9) << ",0"
               << ","
               << "AI,4,3," << std::fixed << std::setprecision(3) << real_signals.at(10) << ",0"
               << ","
               << "DI,4,101," << std::fixed << std::setprecision(0) << real_signals.at(99) << ",1"  // Running
               << ","
               << "DI,4,102," << std::fixed << std::setprecision(0) << real_signals.at(100) << ",1"  // Ready
               << ","
               << "DI,4,121," << 0 << ",1"  // PowerLim

               << "\r\n";

        // socket->send_to(card37, remote_endpoint);
        //  std::cout << card37.str();
        //  std::cout << card38.str();
        //  socket->send_to(boost::asio::buffer(card37.str()), remote_endpoint);
        //  socket->send_to(boost::asio::buffer(card38.str()), remote_endpoint);

        while (socket->available()) {
            size_t bytes_received = socket->receive(boost::asio::buffer(recv_buffer));
            std::string message(
                recv_buffer.begin(),
                recv_buffer.begin() + bytes_received);  // Assumes 8-bit charachters (check NMEA standard)

            // std::cout << message;

            if ('$' != message.front()) {
                std::cout << "Invalid message. No '$' in beginning of message." << std::endl;
                return false;
            }
            if ('\n' != message.back()) {
                std::cout << "Invalid message. No '\\n' at end of message." << std::endl;
                return false;
            }

            std::istringstream ss(message);

            std::string name;
            std::string vesselId;
            std::string dontCare;
            std::string cardId;

            std::string type;
            std::string device;
            std::string code;
            std::string value;
            std::string busNo;

            std::getline(ss, name, ',');
            std::getline(ss, vesselId, ',');
            std::getline(ss, dontCare, ',');
            std::getline(ss, cardId, ',');

            while (std::getline(ss, type, ',')) {
                std::getline(ss, device, ',');
                std::getline(ss, code, ',');
                std::getline(ss, value, ',');
                std::getline(ss, busNo, ',');

                mt_commands[std::make_pair(std::stoi(device), std::stoi(code))] = std::stod(value);

                // std::cout << "Type: " << type << "\t device: " << device << "\t code: " << code << "\t value: " <<
                // value << "\t busNo: " << busNo << std::endl;
            }

            if (cardId == "37") {
                socket->send_to(boost::asio::buffer(card37.str()), remote_endpoint);
            }
            else if (cardId == "38") {
                socket->send_to(boost::asio::buffer(card38.str()), remote_endpoint);
            }
        }

        // Thr 1
        real_signals.at(21) = mt_commands[std::make_pair(1, 202)];
        real_signals.at(22) = mt_commands[std::make_pair(1, 203)];

        // Thr 2
        real_signals.at(23) = mt_commands[std::make_pair(2, 202)];
        real_signals.at(24) = mt_commands[std::make_pair(2, 203)];

        // Thr 3
        real_signals.at(25) = mt_commands[std::make_pair(3, 202)];
        real_signals.at(26) = mt_commands[std::make_pair(3, 203)];

        // Thr 4
        real_signals.at(27) = mt_commands[std::make_pair(4, 202)];
        real_signals.at(28) = mt_commands[std::make_pair(4, 203)];

        return true;
    }

   private:
    // $MTLLF,CARD_ID,DEVICE,CODE,VALUE\n\r

    boost::asio::io_service io_service;
    std::shared_ptr<udp::socket> socket;

    boost::asio::ip::udp::endpoint remote_endpoint;
    boost::asio::ip::udp::endpoint local_endpoint;

    boost::array<char, 65507> recv_buffer;

    std::string _resource_location;

    cppfmu::FMIValueReference remote_ip_vr;
    cppfmu::FMIValueReference remote_port_vr;
    cppfmu::FMIValueReference local_ip_vr;
    cppfmu::FMIValueReference local_port_vr;

    std::unordered_map<cppfmu::FMIValueReference, cppfmu::FMIReal> real_signals;
    std::unordered_map<cppfmu::FMIValueReference, cppfmu::FMIInteger> integer_signals;
    std::unordered_map<cppfmu::FMIValueReference, std::string> string_signals;
    std::unordered_map<cppfmu::FMIValueReference, cppfmu::FMIBoolean> boolean_signals;

    vector<cppfmu::FMIValueReference> input_vr;
    vector<cppfmu::FMIValueReference> output_vr;
    std::map<std::pair<int, int>, double> mt_commands;
};

struct component
{
    std::vector<int> codes;
};

struct mtllc
{
    int cardId;
    std::unordered_map<int, component> components;
};

cppfmu::UniquePtr<cppfmu::SlaveInstance> CppfmuInstantiateSlave(
    cppfmu::FMIString /*instanceName*/, cppfmu::FMIString fmuGUID, cppfmu::FMIString fmuResourceLocation,
    cppfmu::FMIString /*mimeType*/, cppfmu::FMIReal /*timeout*/, cppfmu::FMIBoolean /*visible*/,
    cppfmu::FMIBoolean /*interactive*/, cppfmu::Memory memory, cppfmu::Logger logger)
{
    if (std::strcmp(fmuGUID, FMU_UUID) != 0) {
        throw std::runtime_error("FMU GUID mismatch");
    }
    return cppfmu::AllocateUnique<FmuMtDriver>(memory, fmuResourceLocation);
}
