#include "webserver.h"

namespace beast = boost::beast;         // from <boost/beast.hpp>
namespace http = beast::http;           // from <boost/beast/http.hpp>
namespace websocket = beast::websocket; // from <boost/beast/websocket.hpp>
namespace net = boost::asio;            // from <boost/asio.hpp>
using tcp = boost::asio::ip::tcp;       // from <boost/asio/ip/tcp.hpp>


SystemData *system_data_ptr;
RobotState *robot_state_ptr;
CommandData *commmand_data_ptr;

// -------------- Inconming data Parser --------------------
class ParseData
{
public:
    ParseData() {}

    void parse(const rapidjson::Document &document)
    {
        if (document.HasMember("system_data"))
        {
            const rapidjson::Value &system_data = document["system_data"];
            try
            {
                setSystemData(system_data);
            }
            catch (const std::exception &e)
            {
                std::cerr << e.what() << '\n';
            }
        }

        if (document.HasMember("command_data"))
        {
            const rapidjson::Value &command_data = document["command_data"];
            try
            {
                setCommandData(command_data);
            }
            catch (const std::exception &e)
            {
                std::cerr << e.what() << '\n';
            }
            std::cout << "data read\n";
        }
    }

private:
    void setSystemData(const rapidjson::Value &system_data)
    {
        if (!system_data["power_on"].IsBool())
        {
            throw std::runtime_error("power_on is not a boolean");
        }
        std::cout << "Change power button : " << system_data["power_on"].GetBool() << "\n";
        if (system_data["power_on"].GetBool())
        {
            //
            system_data_ptr->powerOn();
        }
        else
        {
            system_data_ptr->powerOff();
        }
    }

    void setCommandData(const rapidjson::Value &command_data)
    {
        int command_type = command_data["type"].GetInt();
        switch (command_type)
        {
        case CommandType::JOG:
        {
            std::cout << "jog command\n";
            int mode = command_data["mode"].GetInt();
            int index = command_data["index"].GetInt();
            int dir = command_data["direction"].GetInt();
            commmand_data_ptr->setJog(index, dir, mode);
            break;
        }
        case CommandType::HAND_CONTROL:
        {

            std::cout << "Hand Control command\n";
            commmand_data_ptr->setHandControl();
            break;
        }
        case CommandType::GRAVITY:
        {
            std::cout << "Gravity command\n";
            commmand_data_ptr->setGravity();
            break;
        }
        case CommandType::MOVE_TO:
        {
            std::cout << "move command\n";
            double final_pos[6] = {0, 0, 0, 0, 0, 0};
            commmand_data_ptr->setMoveTo(final_pos, 0);
            break;
        }
        default:
            break;
        }
    }
};

// -------------------------------- Serialize outgoing data -------------------------
class SerializeData
{
public:
    SerializeData() : writter(sb) {}

    std::string serialize(rapidjson::Document &document)
    {
        rapidjson::Value &robot_state = document["current_state"];
        setRobotState(robot_state);

        rapidjson::Value &system_data = document["system_state"];
        setSystemState(system_data);
        sb.Clear();
        writter.Reset(sb);
        document.Accept(writter);

        return std::string(sb.GetString());
    }

private:
    rapidjson::StringBuffer sb;
    rapidjson::PrettyWriter<rapidjson::StringBuffer> writter;

    void setRobotState(rapidjson::Value &document)
    {
        rapidjson::Value &position = document["positions"];
        rapidjson::Value &velocity = document["velocities"];
        rapidjson::Value &torque = document["torque"];
        for (rapidjson::SizeType jnt_cnt = 0; jnt_cnt < 6; jnt_cnt++)
        {
            std::string name = "joint" + std::to_string(jnt_cnt + 1);
            position[name.c_str()] = robot_state_ptr->joint_position[jnt_cnt];
            velocity[name.c_str()] = robot_state_ptr->joint_velocity[jnt_cnt];
            torque[name.c_str()] = robot_state_ptr->joint_torque[jnt_cnt];
        }

        rapidjson::Value &tcp_pose = document["tcp_pose"];
        tcp_pose["x"] = robot_state_ptr->cart_position[0];
        tcp_pose["y"] = robot_state_ptr->cart_position[1];
        tcp_pose["z"] = robot_state_ptr->cart_position[2];
        tcp_pose["roll"] = robot_state_ptr->cart_position[3];
        tcp_pose["pitch"] = robot_state_ptr->cart_position[4];
        tcp_pose["yaw"] = robot_state_ptr->cart_position[5];
    }

    void setSystemState(rapidjson::Value &document)
    {
        document["power_on_status"] = system_data_ptr->getSystemState();
    }
};

// Report a failure
void fail(beast::error_code ec, char const *what)
{
    std::cerr << what << ": " << ec.message() << "\n";
}


// ----------------------- Session -----------------------------------
class session : public std::enable_shared_from_this<session>
{
    websocket::stream<beast::tcp_stream> ws_;
    beast::flat_buffer buffer_;
    ParseData data_parser_;
    SerializeData data_serializer_;
    rapidjson::Document data_incoming_;
    rapidjson::Document data_outgoing_;

    std::string outgoing_str_;

public:
    // Take ownership of the socket
    explicit session(tcp::socket &&socket)
        : ws_(std::move(socket))
    {
        // Set outgoing data json
        std::ifstream ifs("../data_template.json");
        // if (ifs.is_open())
        //     std::cout << ifs.rdbuf();
        rapidjson::IStreamWrapper isw(ifs);
        data_outgoing_.ParseStream(isw);
        if(data_outgoing_.HasParseError())
        {
            printf("error while parsing");
            exit(EXIT_FAILURE);
        }
    }

    // Get on the correct executor
    void
    run()
    {
        // We need to be executing within a strand to perform async operations
        // on the I/O objects in this session. Although not strictly necessary
        // for single-threaded contexts, this example code is written to be
        // thread-safe by default.
        net::dispatch(ws_.get_executor(),
                      beast::bind_front_handler(
                          &session::on_run,
                          shared_from_this()));
    }

    // Start the asynchronous operation
    void
    on_run()
    {
        std::cout << "WebSocekt stated...\n";
        // Set suggested timeout settings for the websocket
        ws_.set_option(
            websocket::stream_base::timeout::suggested(
                beast::role_type::server));

        // Set a decorator to change the Server of the handshake
        ws_.set_option(websocket::stream_base::decorator(
            [](websocket::response_type &res)
            {
                res.set(http::field::server,
                        std::string(BOOST_BEAST_VERSION_STRING) +
                            " websocket-server-async");
            }));
        // Accept the websocket handshake
        ws_.async_accept(
            beast::bind_front_handler(
                &session::on_accept,
                shared_from_this()));
    }

    void
    on_accept(beast::error_code ec)
    {
        /*   // uncomment this
        if(ec)
            return fail(ec, "accept");
        */

        // Read a message
        do_read();
        do_write();
    }

    void
    do_read()
    {
        // Read a message into our buffer
        ws_.async_read(
            buffer_,
            beast::bind_front_handler(
                &session::on_read,
                shared_from_this()));
    }

    void
    do_write()
    {
        outgoing_str_.clear();
        outgoing_str_ = data_serializer_.serialize(data_outgoing_);
        // Echo the message
        ws_.text(ws_.got_text());
        ws_.async_write(
            net::buffer(outgoing_str_),
            beast::bind_front_handler(
                &session::on_write,
                shared_from_this()));
    }

    void
    on_read(
        beast::error_code ec,
        std::size_t bytes_transferred)
    {
        boost::ignore_unused(bytes_transferred);

        // This indicates that the session was closed
        if (ec == websocket::error::closed)
            return;

        if (ec)
            return fail(ec, "read");

        std::string rec_string = beast::buffers_to_string(buffer_.data());
        // std::cout << "data :" << rec_string << "\n";
        data_incoming_.Parse<0>(rec_string.c_str()).HasParseError();

        data_parser_.parse(data_incoming_);

        // Clear the buffer
        buffer_.consume(buffer_.size());

        // Do another read
        do_read();
    }

    void
    on_write(
        beast::error_code ec,
        std::size_t bytes_transferred)
    {
        boost::ignore_unused(bytes_transferred);

        if (ec)
            return fail(ec, "write");

        usleep(1000);

        // Do another read
        do_write();
    }
};


// ---------------------------------- Listener ----------------------------
// Accepts incoming connections and launches the sessions
class listener : public std::enable_shared_from_this<listener>
{
    net::io_context &ioc_;
    tcp::acceptor acceptor_;

public:
    listener(
        net::io_context &ioc,
        tcp::endpoint endpoint)
        : ioc_(ioc), acceptor_(ioc)
    {
        beast::error_code ec;

        // Open the acceptor
        acceptor_.open(endpoint.protocol(), ec);
        if (ec)
        {
            fail(ec, "open");
            return;
        }

        // Allow address reuse
        acceptor_.set_option(net::socket_base::reuse_address(true), ec);
        if (ec)
        {
            fail(ec, "set_option");
            return;
        }

        // Bind to the server address
        acceptor_.bind(endpoint, ec);
        if (ec)
        {
            fail(ec, "bind");
            return;
        }

        // Start listening for connections
        acceptor_.listen(
            net::socket_base::max_listen_connections, ec);
        if (ec)
        {
            fail(ec, "listen");
            return;
        }
    }

    // Start accepting incoming connections
    void
    run()
    {
        do_accept();
    }

private:
    void
    do_accept()
    {
        // The new connection gets its own strand
        acceptor_.async_accept(
            net::make_strand(ioc_),
            beast::bind_front_handler(
                &listener::on_accept,
                shared_from_this()));
    }

    void
    on_accept(beast::error_code ec, tcp::socket socket)
    {
        if (ec)
        {
            fail(ec, "accept");
        }
        else
        {
            // Create the session and run it
            std::make_shared<session>(std::move(socket))->run();
        }

        // Accept another connection
        do_accept();
    }
};


int main(int argc, char *argv[])
{
    // Check command line arguments.
    // if (argc != 4)
    // {
    //     std::cerr << "Usage: websocket-server-async <address> <port> <threads>\n"
    //               << "Example:\n"
    //               << "    websocket-server-async 0.0.0.0 8080 1\n";
    //     return EXIT_FAILURE;
    // }

    auto address_in = "0.0.0.0";
    auto port_in = "8080";
    auto thread_in = "1"; 

    /* the size (in bytes) of shared memory object */
    const int SIZE_SimDATA = sizeof(SystemData);
    const int SIZE_RobState = sizeof(RobotState);
    const int SIZE_ComData = sizeof(CommandData);

    /* shared memory file descriptor */
    double shm_fd_SysData;
    double shm_fd_robState;
    double shm_fd_ComData;

    /* open the shared memory object */
    shm_fd_SysData = shm_open("SysData", O_CREAT | O_RDWR, 0666);
    shm_fd_robState = shm_open("RobState", O_CREAT | O_RDWR, 0666);
    shm_fd_ComData = shm_open("ComData", O_CREAT | O_RDWR, 0666);

    ftruncate(shm_fd_SysData, SIZE_SimDATA);
    ftruncate(shm_fd_robState, SIZE_RobState);
    ftruncate(shm_fd_ComData, SIZE_ComData);

    /* memory map the shared memory object */
    system_data_ptr = static_cast<SystemData *>(mmap(0, SIZE_SimDATA, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd_SysData, 0));
    robot_state_ptr = static_cast<RobotState *>(mmap(0, SIZE_RobState, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd_robState, 0));
    commmand_data_ptr = static_cast<CommandData *>(mmap(0, SIZE_ComData, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd_ComData, 0));

    robot_state_ptr->setZero();
    commmand_data_ptr->type = CommandType::NONE;

    auto const address = net::ip::make_address(address_in);
    auto const port = static_cast<unsigned short>(std::atoi(port_in));
    auto const threads = std::max<int>(1, std::atoi(thread_in));

    // The io_context is required for all I/O
    net::io_context ioc{threads};

    // Create and launch a listening port
    std::make_shared<listener>(ioc, tcp::endpoint{address, port})->run();

    // Run the I/O service on the requested number of threads
    std::vector<std::thread> v;
    v.reserve(threads - 1);
    for (auto i = threads - 1; i > 0; --i)
        v.emplace_back(
            [&ioc]
            {
                ioc.run();
            });

    ioc.run();

    return EXIT_SUCCESS;
}