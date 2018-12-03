#pragma once

//#define PR_FUNC() {}
#define PR_FUNC() {printf(__func__); printf("\n");}

#define SAVE_MAX(what, where) do{ if((what) > where) where = (what);}while(0)
#define SAVE_MIN(what, where) do{ if((what) < where) where = (what);}while(0)


