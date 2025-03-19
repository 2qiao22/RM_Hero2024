#pragma once
#include <stdint.h>
#include <cstdint>

class OPERATE{
  private:
    int16_t rx = 0, ry = 0, lx = 0, ly = 0;
    int_fast8_t rs = 0, ls = 0;
    int16_t sr = 0;
    int16_t mx = 0, my = 0;
    int_fast8_t lc = 0, rc = 0;
    uint_fast8_t wk = 0, sk = 0, ak = 0, dk = 0, sh = 0, ct = 0, qk = 0, ek = 0;
    uint_fast8_t rk = 0, fk = 0, gk = 0, zk = 0 ,xk = 0, ck = 0, vk = 0, bk = 0;
    
  public:
    uint_fast8_t available_count = 0;
    void begin();
    void update();
    int16_t RX(){ return rx; }
    inline int16_t      RY(){ return ry; }
    inline int16_t      LX(){ return lx; }
    inline int16_t      LY(){ return ly; }
    inline uint_fast8_t RS(){ return rs; }
    inline uint_fast8_t LS(){ return ls; }
    inline int16_t      SR(){ return sr; }
    inline int16_t      MX(){ return mx; }
    inline int16_t      MY(){ return my;}
    inline uint_fast8_t LC(){ return lc;}
    inline uint_fast8_t RC(){ return rc;}
    inline uint_fast8_t Wk(){ return wk;}
    inline uint_fast8_t Sk(){ return sk;}
    inline uint_fast8_t Ak(){ return ak;}
    inline uint_fast8_t Dk(){ return dk;}
    inline uint_fast8_t Sh(){ return sh;}
    inline uint_fast8_t Ct(){ return ct;}
    inline uint_fast8_t Qk(){ return qk;}
    inline uint_fast8_t Ek(){ return ek;}
    inline uint_fast8_t Rk(){ return rk;}
    inline uint_fast8_t Fk(){ return fk;}
    inline uint_fast8_t Gk(){ return gk;}
    inline uint_fast8_t Zk(){ return zk;}
    inline uint_fast8_t Xk(){ return xk;}
    inline uint_fast8_t Ck(){ return ck;}
    inline uint_fast8_t Vk(){ return vk;}
    inline uint_fast8_t Bk(){ return bk;}
}extern operate;
