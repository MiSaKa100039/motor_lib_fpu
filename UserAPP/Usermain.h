#ifndef USERMAIN_H
#define USERMAIN_H

#ifdef __cplusplus
extern "C" {
#endif

    /* --- 这就是 C 和 C++ 的握手处 --- */
    /* 这个函数在 C 里叫 User_Init，在 C++ 里也叫 User_Init (不因重载改名) */
    void User_Init(void);
    void User_Loop(void);

#ifdef __cplusplus
}
#endif

#endif // USERMAIN_H