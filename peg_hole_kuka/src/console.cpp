
#include "peg_hole_kuka/console.h"
#include "peg_hole_kuka/String_cmd.h"

namespace cmdi {


Console_Interface::Console_Interface(string name, PegConsole* peg_console)
    :Console::Command(name),peg_console(peg_console)
{
}

PegConsole *Console_Interface::GetInterface(){
    return peg_console;
}

int Console_Interface::Execute(string args){
    return peg_console->RespondToConsoleCommand(m_Name,Tokenize(args));
}



PegConsole::PegConsole(ros::NodeHandle &nh, Cmd_interface &cmd_interface):
nh(nh),
cmd_interface(cmd_interface)
{

    service_client = nh.serviceClient<peg_hole_kuka::String_cmd>("add_two_ints");
    service_callback = NULL;

}

void PegConsole::AddConsoleCommand(const std::string& command){
    mConsole.AddCommand(new Console_Interface(command,this));
    list_cmds.push_back(command);

}

int PegConsole::start(){
    mStdout = cout.rdbuf();
    cout.rdbuf(mOutputStream.rdbuf());
    mConsole.SetName("Cmd");

    mNCConsole.SetConsole(&mConsole);
    mNCConsole.InitNCurses();
    mNCConsole.SetTopStaticLinesCount(2);
    return 1;
}

int PegConsole::stop(){
    usleep(100000);
    mNCConsole.FreeNCurses();
    std::cout<< "console stopped" << std::endl;
    return 1;
}

int PegConsole::RespondToConsoleCommand(const string cmd, const vector<string> &args){

    if (std::find(list_cmds.begin(), list_cmds.end(), cmd) != list_cmds.end()){

        //if(service_callback != NULL){
            peg_hole_kuka::String_cmd::Request req;
            peg_hole_kuka::String_cmd::Response res;

            req.cmd = cmd;
           // mConsole.Print("callback: " + req.cmd);
            cmd_interface.service_callback(req,res);

     /*       return (*service_callback)(req,res);
        }else{
            mConsole.Print("service_callback is NULL");
        }*/

    }else{

        mConsole.Print("No such command: " + cmd);
    }

    // command is an action
    /*if(kuka_action_client.has_action(cmd))
    {
        std::string current_action_name = kuka_action_client.current_action_name;
        std::string action_name         = cmd;

        std::cout<< "=== Service call back === " <<                                 std::endl;
        std::cout<< " current action:        "   << current_action_name          << std::endl;
        std::cout<< " requested action:      "   << action_name                  << std::endl;

       if(!kuka_action_client.b_action_running){
           std::cout<< " start action:          "   << action_name << std::endl;
            boost::thread( boost::bind( &ac::Kuka_action_client::call_action, boost::ref(kuka_action_client),action_name ) );
        }else{
            kuka_action_client.ac_.cancelAllGoals();
            kuka_action_client.b_action_running = false;
            worker_thread.join();
        }
    }*/

    // command is a sub-action


    return 1;
}

void PegConsole::addConsole(Console& console){
    mConsole.AddConsole(&console);
}

void PegConsole::addcallback(ros_callback* callback){
    service_callback = callback;
}

void PegConsole::ConsoleUpdate(){
    std::string s    = mOutputStream.str();
    /*std::size_t cpos = 0;
    std::size_t opos = 0;

    if(s.size()>0){
        for(unsigned int i=0;i<s.size();i++){
            opos = cpos;
            cpos = s.find("\n",opos);
            string ss = s.substr(opos,cpos-opos);
            if(ss.size()>0){
                mNCConsole.Print(ss);
            }
            if(cpos==string::npos)
                break;
            cpos++;
        }
        mOutputStream.str("");
    }*/

    int index=0;
    sprintf(static_txt, "-----------------------\n PEG IN HOLE \n-----------------------\n");
    mNCConsole.SetTopStaticLine(index, static_txt);

    mNCConsole.Process();
    mNCConsole.Render();
}

}
