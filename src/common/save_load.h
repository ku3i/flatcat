#ifndef SAVE_LOAD_H_INCLUDED
#define SAVE_LOAD_H_INCLUDED

#include <string>
#include <common/file_io.h>


namespace common {

class Save_Load {
public:

    typedef file_io::CSV_File<double> csv_file_t;
    typedef std::string folder_t;


//    template<typename... Args>
 //   void save(const char* folder_t, const Args&... args)

  //  template<typename... Args>
   // void load(const char* folder_t, const Args&... args)

//    virtual void save(folder_t& f, ... ) = 0;
//    virtual void load(folder_t& f, ... ) = 0;
    virtual ~Save_Load() {}
};

}

#endif /* SAVE_LOAD_H_INCLUDED */
