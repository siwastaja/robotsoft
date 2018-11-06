#pragma once

//#define PR_FUNC() {}
#define PR_FUNC() {printf(__func__); printf("\n");}
