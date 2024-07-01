// Example of using the GeographicLib::MGRS class

#include <iostream>
#include <fstream>
#include <exception>
#include <string>
#include <sstream>
#include <iomanip>
#include <GeographicLib/UTMUPS.hpp>
#include <GeographicLib/MGRS.hpp>

void split(const std::string &sentence_string, std::vector<std::string> &split_str)
{
	std::vector<std::string> str_vec_ptr;
	std::string token1;
	std::stringstream ss1(sentence_string);

	while (getline(ss1, token1, ','))
	{
		std::string token2;
		std::stringstream ss2(token1);
		while(getline(ss2, token2, ';'))
		{
			std::string token3;
			std::stringstream ss3(token2);
			while(getline(ss3, token3, '*'))
				split_str.push_back(token3);
		}
	}
}

//直交座標系を緯度経度座標系に変換
void planeToLatlon(const double x, const double y, const uint8_t plane, double &ido, double &keido)
{
  const double a=6378137, rf=298.257222101, m0=0.9999, s2r=M_PI/648000, n=0.5/(rf-0.5);
  const double n15=1.5*n, anh=0.5*a/(1+n), nsq=n*n, ra=2*anh*m0*(1+nsq/4+nsq*nsq/64);
  const int jt=5, jt2=2*jt;
  double ep=1.0;
  double e[jt2+1], s[jt2+1+1], t[jt2+1], beta[5+1], dlt[6+1];
  s[0] = 0;

  for(int k=1; k<=jt; k++)
  {
    e[k] = n15/k-n;
    ep *= e[k];
    e[k+jt] = n15/(k+jt)-n;
  }

  // 展開パラメータの事前入力
  beta[1]=(1.0/2.0+(-2.0/3.0+(37.0/96.0+(-1.0/360.0-81.0/512.0*n)*n)*n)*n)*n;
  beta[2]=(1.0/48.0+(1.0/15.0+(-437.0/1440.0+46.0/105.0*n)*n)*n)*nsq;
  beta[3]=(17.0/480.0+(-37.0/840.0-209.0/4480.0*n)*n)*n*nsq;
  beta[4]=(4397.0/161280.0-11.0/504.0*n)*nsq*nsq;
  beta[5]=4583.0/161280.0*n*nsq*nsq;
  dlt[1]=(2.0+(-2.0/3.0+(-2.0+(116.0/45.0+(26.0/45.0-2854.0/675.0*n)*n)*n)*n)*n)*n;
  dlt[2]=(7.0/3.0+(-8.0/5.0+(-227.0/45.0+(2704.0/315.0+2323.0/945.0*n)*n)*n)*n)*nsq;
  dlt[3]=(56.0/15.0+(-136.0/35.0+(-1262.0/105.0+73814.0/2835.0*n)*n)*n)*n*nsq;
  dlt[4]=(4279.0/630.0+(-332.0/35.0-399572.0/14175.0*n)*n)*nsq*nsq;
  dlt[5]=(4174.0/315.0-144838.0/6237.0*n)*n*nsq*nsq;
  dlt[6]=601676.0/22275.0*nsq*nsq*nsq;

  // 平面直角座標の座標系原点の緯度を度単位で、経度を分単位で格納
  double phi0[] = { 0,33,33,36,33,36,36,36,36,36,40,44,44,44,26,26,26,26,20,26};
  double lmbd0[] = {0,7770,7860,7930,8010,8060,8160,8230,8310,8390,8450,8415,8535,8655,8520,7650,7440,7860,8160,9240};

  // 該当緯度の 2 倍角の入力により赤道からの子午線弧長を求める関数
  auto Merid = [](const double phi2, const double ep, const double anh, const double *e, double *s, double *t)
  {
    double dc=2.0*std::cos(phi2);
    s[1]=std::sin(phi2);
    for(int i=1; i<=jt2; i++)
    {
      s[i+1]=dc*s[i]-s[i-1];
      t[i]=(1.0/i-4.0*i)*s[i];
    }
    double sum=0.0, c1=ep;
    int j=jt;
    while(j > 0) {
      double c2=phi2, c3=2.0;
      int l=j, m=0;
      while(l > 0)
      {
        c2+=(c3/=e[l--])*t[++m]+(c3*=e[2*j-l])*t[++m];
#if 0					
        c3/=e[l];
        m++;
        c3*=e[2*j-l];
        c2+=(c3)*t[m];
        l--;
        m++;
        c2+=(c3)*t[m];
#endif					
      }
      sum+=c1*c1*c2 ; c1/=e[j];
      j--;
    }
    // << anh << "," << sum << "," << phi2 << "," << ep << std::endl;
    return anh*(sum+phi2);
  };

  //x = 11573.375;
  //y = 22694.380;
  // 実際の計算実行部分
  double xi = (x+m0*Merid(2.0*phi0[plane]*3600.0*s2r,ep,anh,e,s,t))/ra, eta=y/ra, sgmp=1, taup=0;
  //std::cout << "xi," << xi << std::endl;
  double xip = xi, etap = eta;
  for(int j=5; j>0; --j ) {
    double besin=beta[j]*std::sin(2*j*xi), becos=beta[j]*std::cos(2*j*xi);
    xip-=besin*std::cosh(2*j*eta); etap-=becos*std::sinh(2*j*eta);
    sgmp-=2*j*becos*std::cosh(2*j*eta); taup+=2*j*besin*std::sinh(2*j*eta);
  }

  double sxip=std::sin(xip), cxip=std::cos(xip), shetap=std::sinh(etap), chetap=std::cosh(etap);
  double chi=std::asin(sxip/chetap);
  double phi = chi;
  for(int j=6; j>0; --j ) { phi+=dlt[j]*std::sin(2*j*chi); }
  double nphi=(1-n)/(1+n)*std::tan(phi);

  double lmbd=lmbd0[plane]*60.0+std::atan2(shetap, cxip)/s2r;
  double gmm=std::atan2(taup*cxip*chetap+sgmp*sxip*shetap,sgmp*cxip*chetap-taup*sxip*shetap);
  double m=ra/a*std::sqrt((cxip*cxip+shetap*shetap)/(sgmp*sgmp+taup*taup)*(1+nphi*nphi));

  // ラジアン → 度分秒変換
  /*iza=std::floo(phi/s2r/3600);
  ifun =std::floor((phi/s2r-iza*3600)/60);
  ibyou =(phi/s2r-iza*3600-ifun*60)/60.0;
  keiza =std::floor(lmbd/3600);
  keifun =std::floor((lmbd-keiza*3600)/60);
  keibyou =(lmbd-keiza*3600-keifun*60)/60.0;*/
  ido = phi/s2r/3600;
  keido = lmbd/3600;
  double sgn=(gmm<0);
  double gdo=std::floor(gmm/s2r/3600)+sgn;
  double gfun=std::floor((gmm/s2r-gdo*3600)/60)+sgn;
  double gbyou=gmm/s2r-gdo*3600-gfun*60;

//		std::cout << "phi," << phi/s2r/3600 << "  lmd," << keido << std::endl;
  //std::cout << "phi=" << iza << "°" << ifun << "'" << ibyou << "\"  lmd=" << keiza << "°" << keifun << "'" << keibyou << "\"" << std::endl;
}

int main() {
  double x = 17283.0669;
  double y = 7965.2214;
  double lat, lon;
  planeToLatlon(x, y, 9, lat, lon);
  std::cout << lat << ',' << lon << std::endl;
  return 0;


  /*std::ifstream ifs("/home/sit/ダウンロード/8940baeef132a768.csv");
  while(!ifs.eof())
  {
    std::string line;
    std::getline(ifs, line);
    std::vector<std::string> parts;
    split(line, parts);

    if(parts.size() >= 3)
    {
      double lat = std::stod(parts[0]);
      double lon = std::stod(parts[1]);
      int zone;
      bool northp;
      double x, y;
      GeographicLib::UTMUPS::Forward(lat, lon, zone, northp, x, y);
      std::string mgrs;
      //std::cout << x << ',' << y << std::endl;
      GeographicLib::MGRS::Forward(zone, northp, x, y, lat, 10, mgrs);
      //std::cout << mgrs << std::endl;

      std::string header = mgrs.substr(0, 5);
      double mgrs_x = std::stod(mgrs.substr(5, 10)) / 1E5;
      double mgrs_y = std::stod(mgrs.substr(15, 10)) / 1E5;
      std::cout << header << ',' << std::fixed << std::setprecision(8) << mgrs_x << ',' << mgrs_y << ',' <<  parts[2] << std::endl;
    }
  }

  ifs.close();
  return 0;*/


  /*try {
    // See also example-GeoCoords.cpp
    {
      // Sample forward calculation
      double lat = 35.408404548481606, lon = 138.20803635718397; // Baghdad
      int zone;
      bool northp;
      double x, y;
      UTMUPS::Forward(lat, lon, zone, northp, x, y);
      string mgrs;
      cout << x << ',' << y << std::endl;
      MGRS::Forward(zone, northp, x, y, lat, 5, mgrs);
      cout << mgrs << "\n";
    }
    {
      // Sample reverse calculation
      string mgrs = "54STE4646721915";
      int zone, prec;
      bool northp;
      double x, y;
      MGRS::Reverse(mgrs, zone, northp, x, y, prec);
      double lat, lon;
      UTMUPS::Reverse(zone, northp, x, y, lat, lon);
      cout << prec << " " << lat << " " << lon << "\n";
    }
  }
  catch (const exception& e) {
    cerr << "Caught exception: " << e.what() << "\n";
    return 1;
  }*/
}