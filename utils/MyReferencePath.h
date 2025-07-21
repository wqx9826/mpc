//
// Created by chh3213 on 2022/11/24.
//

#ifndef CHHROBOTICS_CPP_MYREFERENCEPATH_H
#define CHHROBOTICS_CPP_MYREFERENCEPATH_H
#include  "data_struct.h"


class MyReferencePath {

public:
    MyReferencePath();

    void setReferencePath();



    std::vector<Trajponits> refer_path;
};


#endif //CHHROBOTICS_CPP_MYREFERENCEPATH_H
