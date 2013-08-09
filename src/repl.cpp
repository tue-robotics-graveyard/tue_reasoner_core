#include <reasoner/Reasoner.h>

#include <SWI-Prolog.h>

using namespace std;

int main(int argc, char **argv) {

    Reasoner r;

    PL_install_readline();

   for(;;)
   { int status = PL_toplevel() ? 0 : 1;

     PL_halt(status);
   }
}
