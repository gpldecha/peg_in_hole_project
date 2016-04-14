#ifndef PEG_HOLE_KUKA__CONSOLE_H_
#define PEG_HOLE_KUKA__CONSOLE_H_

#include "lwr_console/Console.h"
#include "lwr_console/NCConsole.h"
#include "lwr_console/Various.h"
#include "peg_hole_kuka/String_cmd.h"
#include "peg_hole_kuka/cmd_interface.h"


#include <functional>

#include <ros/ros.h>
#include <boost/thread.hpp>

namespace cmdi {

typedef std::function<bool(peg_hole_kuka::String_cmd::Request& req,peg_hole_kuka::String_cmd::Response &res)> ros_callback;

class PegConsole;

class Console_Interface : public Console::Command {

public:

    Console_Interface(string name, PegConsole *peg_console);

    PegConsole*    GetInterface();

    virtual int Execute(string args);

protected:

    PegConsole  *peg_console;

};

class PegConsole{

public:

    PegConsole(ros::NodeHandle& nh, Cmd_interface& cmd_interface);

    int  stop();

    int  start();

    int  RespondToConsoleCommand(const string cmd, const vector<string> &args);

    void AddConsoleCommand(const std::string& command);

    void addConsole(Console& console);

    void ConsoleUpdate();

    void addcallback(ros_callback* callback);

    Console *get_console();


private:

    ros_callback*                       service_callback;
    Cmd_interface&                      cmd_interface;

    ros::ServiceClient                  service_client;

    std::vector<std::string>            list_cmds;

    ros::NodeHandle&                    nh;
    NCConsole                           mNCConsole;
    Console                             mConsole;

    std::vector<ros::Subscriber>        subs;
    boost::thread                       worker_thread;
    ros::ServiceServer                  action_service;
    ros::ServiceServer                  cmd_interface_service;

    streambuf                           *mStdout;
    stringstream                        mOutputStream;
    char                                static_txt[1025];

};


}


#endif
