#ifndef __BUTTERWORTH_H__
#define __BUTTERWORTH_H__

//Low pass butterworth filter order=2 alpha1=0.1
class  butterworth
{
  public:
    butterworth()
    {
      v[0]=0.0;
      v[1]=0.0;
    }
  private:
    float v[3];
  public:
    float step(float x) //class II
    {
      v[0] = v[1];
      v[1] = v[2];
      v[2] = (6.745527388907189559e-2 * x)
         + (-0.41280159809618854894 * v[0])
         + (1.14298050253990091107 * v[1]);
      return
         (v[0] + v[2])
        +2 * v[1];
    }
};

class  butterworth2
{
  public:
    butterworth2()
    {
      v[0]=0.0;
      v[1]=0.0;
    }
  private:
    float v[3];
  public:
    float step(float x) //class II
    {
      v[0] = v[1];
      v[1] = v[2];
      v[2] = (9.865221073379615291e-8 * x)
         + (-0.99911181807963878043 * v[0])
         + (1.99911142347079584525 * v[1]);
         /*
         v[2] = (2.466853082916387383e-8 * x)
         + (-0.99955581038761343038 * v[0])
         + (1.99955571171349011372 * v[1]);*/
      return
         (v[0] + v[2])
        +2 * v[1];
    }
};

#endif
