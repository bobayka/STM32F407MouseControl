
/*
http://robotclass.ru/articles/magnetometer-and-compass/
http://www.avislab.com/blog/hmc5883l_ru/
https://www.st.com/content/ccc/resource/technical/document/application_note/e6/f0/fa/af/94/5e/43/de/CD00269797.pdf/files/CD00269797.pdf/jcr:content/translations/en.CD00269797.pdf
*/


#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "Magnitometer_Calibration.h"  


#define X 0
#define Y 1
#define Z 2
float data_offs_max[3]={-50 ,0 ,-50};
float data_offs_min[3]={0};
float xmoc=0,ymoc=0,zmoc=0;//раскоментить,если используем данную функцию
float kx = 1, ky = 1, kz = 1;

void Magnitometer_Calibration (struct Magnet*  magndata )  {
    static float max=0,min=0;
    float mx = magndata->x; 
		float my = magndata->y;
		float mz = magndata->z;
		static int cnt=0;

    if (cnt < 3)
        cnt++;
    else if(cnt<2500){//жду минуту чтобы откалибровать акселлерометр

      ++cnt;
      
      if(mx>data_offs_max[X])
          data_offs_max[X]=mx;
      
      if(my>data_offs_max[Y])
          data_offs_max[Y]=my;
      
      if(mz>data_offs_max[Z])
          data_offs_max[Z]=mz;
      
      if(mx<data_offs_min[X])
          data_offs_min[X]=mx;
      
      if(my<data_offs_min[Y])
          data_offs_min[Y]=my;
      
      if(mz<data_offs_min[Z])
          data_offs_min[Z]=mz;
            
      }
      else if(cnt==2500){
        cnt++;

        xmoc = (data_offs_min[X]-data_offs_max[X])/2-data_offs_min[X];
        ymoc = (data_offs_min[Y]-data_offs_max[Y])/2-data_offs_min[Y];
        zmoc = (data_offs_min[Z]-data_offs_max[Z])/2-data_offs_min[Z];
        
        max=data_offs_max[X];
        if(data_offs_max[Y]>max)
          max=data_offs_max[Y];
        if(data_offs_max[Z]>max)
          max=data_offs_max[Z];
        
        min=data_offs_min[X];              
        if(data_offs_min[Y]<min)
          min=data_offs_min[Y];
        if(data_offs_min[Z]<min)
          min=data_offs_min[Z];
        
        kx=(max-min)/(data_offs_max[X]-data_offs_min[X]);
        ky=(max-min)/(data_offs_max[Y]-data_offs_min[Y]);
        kz=(max-min)/(data_offs_max[Z]-data_offs_min[Z]);
      }
        magndata->x = kx*(mx+xmoc);
        magndata->y = ky*(my+ymoc);
        magndata->z = kz*(mz+zmoc);
      }
