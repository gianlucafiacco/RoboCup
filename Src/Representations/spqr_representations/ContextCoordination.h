
#pragma once

#include "Tools/Enum.h"

STREAMABLE(ContextCoordination,
{
    public:
        ENUM(SpqrRole,
        {,
               goalie = 1,
               defender,
               jolly,
               supporter,
               striker,
               searcher_1,
               searcher_2,
               searcher_3,
               throwin_searcher_1,
               throwin_searcher_2,
               throwin_searcher_3,
               guard,
               no_role, //13
        });

        SpqrRole robotRole,
        ContextCoordination(){ robotRole = SpqrRole::no_role }
});


//this representation should multicast for the i-th
//robot its current role and the time since this role has be set
/// TODO but evaluate whether it worths
