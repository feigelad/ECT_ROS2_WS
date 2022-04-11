#include <proj_api.h>
#include "rtk.h"
#include "../../common/quaternion/quaternion_base.h"

namespace nirvana
{
  namespace location
  {
    bool Rtk::wgs84ToUTM(double lat, double lon, double &x, double &y)
    {
      int nUTMZone = (int)((lon + 186.0) / 6.0);
      bool bNorth = lat > 0 ? true : false;
      if (utm_zone_ != nUTMZone || is_loc_north_ != bNorth)
      {
        //"+proj=utm +zone=50 +datum=WGS84 +units=m +no_defs";
        // "+proj=utm +zone=48 +south +datum=WGS84 +units=m +no_defs"
        std::string qstrUTM = "+proj=utm +zone=" + std::to_string(nUTMZone);
        if (!bNorth)
        {
          qstrUTM += " +south ";
        }
        qstrUTM += " +datum=WGS84 +units=m +no_defs";
        if (pj_utm_ != NULL)
        {
          pj_free(pj_utm_);
          pj_utm_ = NULL;
        }
        std::string strUTM = qstrUTM;
        const char *pUTM = strUTM.c_str();

        pj_utm_ = pj_init_plus(pUTM);

        utm_zone_ = nUTMZone;
        is_loc_north_ = bNorth;
      }
      double xx, yy;
      xx = DegToRad(lon);
      yy = DegToRad(lat);
      pj_transform(pj_wgs84_, pj_utm_, 1, 1, &xx, &yy, NULL);

      x = xx;
      y = yy;
      return true;
    }

    bool Rtk::UTMTowgs84(double x, double y, double &lat, double &lon)
    {
      if (pj_utm_ == NULL)
      {
        return false;
      }
      double longitude, latitude;
      longitude = x;
      latitude = y;
      pj_transform(pj_utm_, pj_wgs84_, 1, 1, &longitude, &latitude, NULL);
      lon = RadToDeg(longitude);
      lat = RadToDeg(latitude);
      return true;
    }

    bool Rtk::XY2BL(double x, double y, double &lat, double &lon)
    {
      if (projType == 'u')
      {
        y = y / 0.9996;
      }

      double bf0 = y / ellipPmt.a0, bf;
      double threshould = 1.0;
      while (threshould > 0.00000001)
      {
        double y0 = -ellipPmt.a2 * sin(2 * bf0) / 2 + ellipPmt.a4 * sin(4 * bf0) / 4 - ellipPmt.a6 * sin(6 * bf0) / 6;
        bf = (y - y0) / ellipPmt.a0;
        threshould = bf - bf0;
        bf0 = bf;
      }

      double t, j2;
      t = tan(bf);
      j2 = ellipPmt.ep2 * pow(cos(bf), 2);

      double v, n, m;
      v = sqrt(1 - ellipPmt.e2 * sin(bf) * sin(bf));
      n = ellipPmt.a / v;
      m = ellipPmt.a * (1 - ellipPmt.e2) / pow(v, 3);

      x = x - 500000;
      if (projType == 'u')
      {
        x = x / 0.9996;
      }

      double temp0, temp1, temp2;
      temp0 = t * x * x / (2 * m * n);
      temp1 = t * (5 + 3 * t * t + j2 - 9 * j2 * t * t) * pow(x, 4) / (24 * m * pow(n, 3));
      temp2 = t * (61 + 90 * t * t + 45 * pow(t, 4)) * pow(x, 6) / (720 * pow(n, 5) * m);
      lat = (bf - temp0 + temp1 - temp2) * 57.29577951308232;

      temp0 = x / (n * cos(bf));
      temp1 = (1 + 2 * t * t + j2) * pow(x, 3) / (6 * pow(n, 3) * cos(bf));
      temp2 = (5 + 28 * t * t + 6 * j2 + 24 * pow(t, 4) + 8 * t * t * j2) * pow(x, 5) / (120 * pow(n, 5) * cos(bf));
      lon = (temp0 - temp1 + temp2) * 57.29577951308232 + meridianLine;

      return true;
    }

    bool Rtk::BL2XY(double lat, double lon, double &x, double &y)
    {
      if (meridianLine < -180)
      {
        meridianLine = int((lon + 1.5) / 3) * 3;
      }

      lat = lat * 0.0174532925199432957692;
      double dL = (lon - meridianLine) * 0.0174532925199432957692;

      double X = ellipPmt.a0 * lat - ellipPmt.a2 * sin(2 * lat) / 2 + ellipPmt.a4 * sin(4 * lat) / 4 - ellipPmt.a6 * sin(6 * lat) / 6;
      double tn = tan(lat);
      double tn2 = tn * tn;
      double tn4 = tn2 * tn2;

      double j2 = (1 / pow(1 - ellipPmt.f, 2) - 1) * pow(cos(lat), 2);
      double n = ellipPmt.a / sqrt(1.0 - ellipPmt.e2 * sin(lat) * sin(lat));

      double temp[6] = {0};
      temp[0] = n * sin(lat) * cos(lat) * dL * dL / 2;
      temp[1] = n * sin(lat) * pow(cos(lat), 3) * (5 - tn2 + 9 * j2 + 4 * j2 * j2) * pow(dL, 4) / 24;
      temp[2] = n * sin(lat) * pow(cos(lat), 5) * (61 - 58 * tn2 + tn4) * pow(dL, 6) / 720;
      temp[3] = n * cos(lat) * dL;
      temp[4] = n * pow(cos(lat), 3) * (1 - tn2 + j2) * pow(dL, 3) / 6;
      temp[5] = n * pow(cos(lat), 5) * (5 - 18 * tn2 + tn4 + 14 * j2 - 58 * tn2 * j2) * pow(dL, 5) / 120;

      y = X + temp[0] + temp[1] + temp[2];
      x = temp[3] + temp[4] + temp[5];

      if (projType == 'g')
      {
        x = x + 500000;
      }
      else if (projType == 'u')
      {
        x = x * 0.9996 + 500000;
        y = y * 0.9996;
      }

      return true;
    }

    bool Rtk::XYZ2BLH(double x, double y, double z, double &lat, double &lon, double &ht)
    {
      double preB, preN;
      double nowB = 0, nowN = 0;
      double threshould = 1.0;

      preB = atan(z / sqrt(x * x + y * y));
      preN = ellipPmt.a / sqrt(1 - ellipPmt.e2 * sin(preB) * sin(preB));
      while (threshould > 0.0000000001)
      {
        nowN = ellipPmt.a / sqrt(1 - ellipPmt.e2 * sin(preB) * sin(preB));
        nowB = atan((z + preN * ellipPmt.e2 * sin(preB)) / sqrt(x * x + y * y));

        threshould = fabs(nowB - preB);
        preB = nowB;
        preN = nowN;
      }
      ht = sqrt(x * x + y * y) / cos(nowB) - nowN;
      lon = atan2(y, x) * 57.29577951308232; // 180 / pi
      lat = nowB * 57.29577951308232;

      return true;
    }

    bool Rtk::BLH2XYZ(double lat, double lon, double ht, double &x, double &y, double &z)
    {
      double sinB = sin(lat / 57.29577951308232);
      double cosB = cos(lat / 57.29577951308232);
      double sinL = sin(lon / 57.29577951308232);
      double cosL = cos(lon / 57.29577951308232);

      double N = ellipPmt.a / sqrt(1.0 - ellipPmt.e2 * sinB * sinB);
      x = (N + ht) * cosB * cosL;
      y = (N + ht) * cosB * sinL;
      z = (N * ellipPmt.b * ellipPmt.b / (ellipPmt.a * ellipPmt.a) + ht) * sinB;

      return true;
    }

    double Rtk::ArcLengthOfMeridian(double phi)
    {
      double alpha, beta, gamma, delta, epsilon, n;
      double result;
      /* Precalculate n */
      n = (SM_A - SM_B) / (SM_A + SM_B);
      /* Precalculate alpha */
      alpha = ((SM_A + SM_B) / 2.0) * (1.0 + (pow(n, 2.0) / 4.0) + (pow(n, 4.0) / 64.0));
      /* Precalculate beta */
      beta = (-3.0 * n / 2.0) + (9.0 * pow(n, 3.0) / 16.0) + (-3.0 * pow(n, 5.0) / 32.0);
      /* Precalculate gamma */
      gamma = (15.0 * pow(n, 2.0) / 16.0) + (-15.0 * pow(n, 4.0) / 32.0);
      /* Precalculate delta */
      delta = (-35.0 * pow(n, 3.0) / 48.0) + (105.0 * pow(n, 5.0) / 256.0);
      /* Precalculate epsilon */
      epsilon = (315.0 * pow(n, 4.0) / 512.0);
      /* Now calculate the sum of the series and return */
      result = alpha * (phi + (beta * sin(2.0 * phi)) + (gamma * sin(4.0 * phi)) + (delta * sin(6.0 * phi)) + (epsilon * sin(8.0 * phi)));
      return result;
    }

    double Rtk::FootpointLatitude(double y)
    {
      double y_, alpha_, beta_, gamma_, delta_, epsilon_, n;
      double result;
      /* Precalculate n (Eq. 10.18) */
      n = (SM_A - SM_B) / (SM_A + SM_B);
      /* Precalculate alpha_ (Eq. 10.22) */
      /* (Same as alpha in Eq. 10.17) */
      alpha_ = ((SM_A + SM_B) / 2.0) * (1 + (pow(n, 2.0) / 4) + (pow(n, 4.0) / 64));
      /* Precalculate y_ (Eq. 10.23) */
      y_ = y / alpha_;
      /* Precalculate beta_ (Eq. 10.22) */
      beta_ = (3.0 * n / 2.0) + (-27.0 * pow(n, 3.0) / 32.0) + (269.0 * pow(n, 5.0) / 512.0);
      /* Precalculate gamma_ (Eq. 10.22) */
      gamma_ = (21.0 * pow(n, 2.0) / 16.0) + (-55.0 * pow(n, 4.0) / 32.0);
      /* Precalculate delta_ (Eq. 10.22) */
      delta_ = (151.0 * pow(n, 3.0) / 96.0) + (-417.0 * pow(n, 5.0) / 128.0);
      /* Precalculate epsilon_ (Eq. 10.22) */
      epsilon_ = (1097.0 * pow(n, 4.0) / 512.0);
      /* Now calculate the sum of the series (Eq. 10.21) */
      result = y_ + (beta_ * sin(2.0 * y_)) + (gamma_ * sin(4.0 * y_)) + (delta_ * sin(6.0 * y_)) + (epsilon_ * sin(8.0 * y_));
      return result;
    }

    void Rtk::MapLatLonToXY(double phi, double lambda, double lambda0, UTMCoor &xy)
    {
      double N, nu2, ep2, t, t2, l;
      double l3coef, l4coef, l5coef, l6coef, l7coef, l8coef;
      double tmp;
      /* Precalculate ep2 */
      ep2 = (pow(SM_A, 2.0) - pow(SM_B, 2.0)) / pow(SM_B, 2.0);
      /* Precalculate nu2 */
      nu2 = ep2 * pow(cos(phi), 2.0);
      /* Precalculate N */
      N = pow(SM_A, 2.0) / (SM_B * sqrt(1 + nu2));
      /* Precalculate t */
      t = tan(phi);
      t2 = t * t;
      tmp = (t2 * t2 * t2) - pow(t, 6.0);
      /* Precalculate l */
      l = lambda - lambda0;
      /* Precalculate coefficients for l**n in the equations below
          so a normal human being can read the expressions for easting
          and northing
          -- l**1 and l**2 have coefficients of 1.0 */
      l3coef = 1.0 - t2 + nu2;
      l4coef = 5.0 - t2 + 9 * nu2 + 4.0 * (nu2 * nu2);
      l5coef = 5.0 - 18.0 * t2 + (t2 * t2) + 14.0 * nu2 - 58.0 * t2 * nu2;
      l6coef = 61.0 - 58.0 * t2 + (t2 * t2) + 270.0 * nu2 - 330.0 * t2 * nu2;
      l7coef = 61.0 - 479.0 * t2 + 179.0 * (t2 * t2) - (t2 * t2 * t2);
      l8coef = 1385.0 - 3111.0 * t2 + 543.0 * (t2 * t2) - (t2 * t2 * t2);
      /* Calculate easting (x) */
      xy.x = N * cos(phi) * l + (N / 6.0 * pow(cos(phi), 3.0) * l3coef * pow(l, 3.0)) + (N / 120.0 * pow(cos(phi), 5.0) * l5coef * pow(l, 5.0)) + (N / 5040.0 * pow(cos(phi), 7.0) * l7coef * pow(l, 7.0));
      /* Calculate northing (y) */
      xy.y = ArcLengthOfMeridian(phi) + (t / 2.0 * N * pow(cos(phi), 2.0) * pow(l, 2.0)) + (t / 24.0 * N * pow(cos(phi), 4.0) * l4coef * pow(l, 4.0)) + (t / 720.0 * N * pow(cos(phi), 6.0) * l6coef * pow(l, 6.0)) + (t / 40320.0 * N * pow(cos(phi), 8.0) * l8coef * pow(l, 8.0));
    }

    void Rtk::MapXYToLatLon(double x, double y, double lambda0, WGS84Corr &philambda)
    {
      double phif, Nf, Nfpow, nuf2, ep2, tf, tf2, tf4, cf;
      double x1frac, x2frac, x3frac, x4frac, x5frac, x6frac, x7frac, x8frac;
      double x2poly, x3poly, x4poly, x5poly, x6poly, x7poly, x8poly;
      /* Get the value of phif, the footpoint latitude. */
      phif = FootpointLatitude(y);
      /* Precalculate ep2 */
      ep2 = (pow(SM_A, 2.0) - pow(SM_B, 2.0)) / pow(SM_B, 2.0);
      /* Precalculate cos (phif) */
      cf = cos(phif);
      /* Precalculate nuf2 */
      nuf2 = ep2 * pow(cf, 2.0);
      /* Precalculate Nf and initialize Nfpow */
      Nf = pow(SM_A, 2.0) / (SM_B * sqrt(1 + nuf2));
      Nfpow = Nf;
      /* Precalculate tf */
      tf = tan(phif);
      tf2 = tf * tf;
      tf4 = tf2 * tf2;
      /* Precalculate fractional coefficients for x**n in the equations
          below to simplify the expressions for latitude and longitude. */
      x1frac = 1.0 / (Nfpow * cf);
      Nfpow *= Nf; /* now equals Nf**2) */
      x2frac = tf / (2.0 * Nfpow);
      Nfpow *= Nf; /* now equals Nf**3) */
      x3frac = 1.0 / (6.0 * Nfpow * cf);
      Nfpow *= Nf; /* now equals Nf**4) */
      x4frac = tf / (24.0 * Nfpow);
      Nfpow *= Nf; /* now equals Nf**5) */
      x5frac = 1.0 / (120.0 * Nfpow * cf);
      Nfpow *= Nf; /* now equals Nf**6) */
      x6frac = tf / (720.0 * Nfpow);
      Nfpow *= Nf; /* now equals Nf**7) */
      x7frac = 1.0 / (5040.0 * Nfpow * cf);
      Nfpow *= Nf; /* now equals Nf**8) */
      x8frac = tf / (40320.0 * Nfpow);
      /* Precalculate polynomial coefficients for x**n.
          -- x**1 does not have a polynomial coefficient. */
      x2poly = -1.0 - nuf2;
      x3poly = -1.0 - 2 * tf2 - nuf2;
      x4poly = 5.0 + 3.0 * tf2 + 6.0 * nuf2 - 6.0 * tf2 * nuf2 - 3.0 * (nuf2 * nuf2) - 9.0 * tf2 * (nuf2 * nuf2);
      x5poly = 5.0 + 28.0 * tf2 + 24.0 * tf4 + 6.0 * nuf2 + 8.0 * tf2 * nuf2;
      x6poly = -61.0 - 90.0 * tf2 - 45.0 * tf4 - 107.0 * nuf2 + 162.0 * tf2 * nuf2;
      x7poly = -61.0 - 662.0 * tf2 - 1320.0 * tf4 - 720.0 * (tf4 * tf2);
      x8poly = 1385.0 + 3633.0 * tf2 + 4095.0 * tf4 + 1575 * (tf4 * tf2);
      /* Calculate latitude */
      philambda.lat = phif + x2frac * x2poly * (x * x) + x4frac * x4poly * pow(x, 4.0) + x6frac * x6poly * pow(x, 6.0) + x8frac * x8poly * pow(x, 8.0);
      /* Calculate longitude */
      philambda.log = lambda0 + x1frac * x + x3frac * x3poly * pow(x, 3.0) + x5frac * x5poly * pow(x, 5.0) + x7frac * x7poly * pow(x, 7.0);
    }

    void Rtk::LatLonToUTMXY(double lat, double lon, int zone, UTMCoor &xy)
    {
      MapLatLonToXY(lat, lon, UTMCentralMeridian(zone), xy);
      /* Adjust easting and northing for UTM system. */
      xy.x = xy.x * UTMScaleFactor + 500000.0;
      xy.y = xy.y * UTMScaleFactor;
      if (xy.y < 0.0)
        xy.y += 10000000.0;
    }

    void Rtk::UTMXYToLatLon(double x, double y, int zone, bool southhemi, WGS84Corr &latlon)
    {
      double cmeridian;
      x -= 500000.0;
      x /= UTMScaleFactor;
      /* If in southern hemisphere, adjust y accordingly. */
      if (southhemi)
        y -= 10000000.0;
      y /= UTMScaleFactor;
      cmeridian = UTMCentralMeridian(zone);
      MapXYToLatLon(x, y, cmeridian, latlon);
    }

    Eigen::Quaterniond Rtk::euler2Quaternion(const double roll_rad, const double pitch_rad, const double yaw_rad)
    {
      Eigen::AngleAxisd rollAngle(roll_rad, Eigen::Vector3d::UnitZ());
      Eigen::AngleAxisd yawAngle(yaw_rad, Eigen::Vector3d::UnitY());
      Eigen::AngleAxisd pitchAngle(pitch_rad, Eigen::Vector3d::UnitX());

      Eigen::Quaterniond q = rollAngle * yawAngle * pitchAngle;
      return q;
    }
    Eigen::Vector3d Rtk::Quaterniond2Euler(const double x, const double y, const double z, const double w)
    {
      Eigen::Quaterniond q;
      q.x() = x;
      q.y() = y;
      q.z() = z;
      q.w() = w;
      Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(2, 1, 0);
      return euler;
    }
    Eigen::Matrix3d Rtk::Quaternion2RotationMatrix(const double x, const double y, const double z, const double w)
    {
      Eigen::Quaterniond q;
      q.x() = x;
      q.y() = y;
      q.z() = z;
      q.w() = w;
      Eigen::Matrix3d R = q.normalized().toRotationMatrix();
      return R;
    }
    Eigen::Quaterniond Rtk::rotationMatrix2Quaterniond(Eigen::Matrix3d R)
    {
      Eigen::Quaterniond q = Eigen::Quaterniond(R);
      q.normalize();
      return q;
    }
    Eigen::Matrix3d Rtk::euler2RotationMatrix(const double roll_rad, const double pitch_rad, const double yaw_rad)
    {
      Eigen::AngleAxisd rollAngle(roll_rad, Eigen::Vector3d::UnitZ());
      Eigen::AngleAxisd yawAngle(yaw_rad, Eigen::Vector3d::UnitY());
      Eigen::AngleAxisd pitchAngle(pitch_rad, Eigen::Vector3d::UnitX());
      Eigen::Quaterniond q = rollAngle * yawAngle * pitchAngle;
      Eigen::Matrix3d R = q.matrix();
      return R;
    }
    Eigen::Vector3d Rtk::RotationMatrix2euler(Eigen::Matrix3d R)
    {
      Eigen::Matrix3d m;
      m = R;
      Eigen::Vector3d euler = m.eulerAngles(0, 1, 2);
      return euler;
    }
  } // namespace location
} // namespace nirvana