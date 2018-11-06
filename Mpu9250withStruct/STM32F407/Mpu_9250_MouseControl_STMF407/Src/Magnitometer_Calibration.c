
/*
http://robotclass.ru/articles/magnetometer-and-compass/
http://www.avislab.com/blog/hmc5883l_ru/
https://www.st.com/content/ccc/resource/technical/document/application_note/e6/f0/fa/af/94/5e/43/de/CD00269797.pdf/files/CD00269797.pdf/jcr:content/translations/en.CD00269797.pdf
*/


#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "Magnitometer_Calibration.h"  

#define x 0
#define y 1
#define z 2
float data_offs_max[3]={-50 ,0 ,-50};
float data_offs_min[3]={0};
int cnt=0;
float xmoc=0,ymoc=0,zmoc=0;//раскоментить,если используем данную функцию
float kx = 1, ky = 1, kz = 1;

void Magnitometer_Calibration (float* Mx, float* My, float* Mz)  {
    static float max=0,min=0;
    float mx = *Mx, my = *My, mz = *Mz;
    if (cnt < 3)
        cnt++;
    else if(cnt<5000){//жду минуту чтобы откалибровать акселлерометр

      ++cnt;
      
      if(mx>data_offs_max[x])
          data_offs_max[x]=mx;
      
      if(my>data_offs_max[y])
          data_offs_max[y]=my;
      
      if(mz>data_offs_max[z])
          data_offs_max[z]=mz;
      
      if(mx<data_offs_min[x])
          data_offs_min[x]=mx;
      
      if(my<data_offs_min[y])
          data_offs_min[y]=my;
      
      if(mz<data_offs_min[z])
          data_offs_min[z]=mz;
            
      }
      else if(cnt==5000){
        cnt++;

        xmoc = (data_offs_min[x]-data_offs_max[x])/2-data_offs_min[x];
        ymoc = (data_offs_min[y]-data_offs_max[y])/2-data_offs_min[y];
        zmoc = (data_offs_min[z]-data_offs_max[z])/2-data_offs_min[z];
        
        max=data_offs_max[x];
        if(data_offs_max[y]>max)
          max=data_offs_max[y];
        if(data_offs_max[z]>max)
          max=data_offs_max[z];
        
        min=data_offs_min[x];              
        if(data_offs_min[y]<min)
          min=data_offs_min[y];
        if(data_offs_min[z]<min)
          min=data_offs_min[z];
        
        kx=(max-min)/(data_offs_max[x]-data_offs_min[x]);
        ky=(max-min)/(data_offs_max[y]-data_offs_min[y]);
        kz=(max-min)/(data_offs_max[z]-data_offs_min[z]);
      }
        *Mx=kx*(mx+xmoc);
        *My=ky*(my+ymoc);
        *Mz=kz*(mz+zmoc);
      }
