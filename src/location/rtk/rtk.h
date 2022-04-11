#pragma once

#include <mutex>
#include <math.h>
#include <proj_api.h>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Core>
#include <stdlib.h>
#include <vector>
#include <math.h>

namespace nirvana
{
  namespace location
  {
    struct EllipsoidParameter
    {
      double a, b, f;
      double e2, ep2;

      // 高斯投影参数
      double c;
      double a0, a2, a4, a6;

      EllipsoidParameter()
      {
        // Default: wgs84
        a = 6378137.0;
        e2 = 0.00669437999013;

        b = sqrt(a * a * (1 - e2));
        ep2 = (a * a - b * b) / (b * b);
        f = (a - b) / a;
        double f0 = 1 / 298.257223563;
        double f1 = 1 / f;

        c = a / (1 - f);
        double m0, m2, m4, m6, m8;
        m0 = a * (1 - e2);
        m2 = 1.5 * e2 * m0;
        m4 = 1.25 * e2 * m2;
        m6 = 7 * e2 * m4 / 6;
        m8 = 9 * e2 * m6 / 8;
        a0 = m0 + m2 / 2 + 3 * m4 / 8 + 5 * m6 / 16 + 35 * m8 / 128;
        a2 = m2 / 2 + m4 / 2 + 15 * m6 / 32 + 7 * m8 / 16;
        a4 = m4 / 8 + 3 * m6 / 16 + 7 * m8 / 32;
        a6 = m6 / 32 + m8 / 16;
      }

      EllipsoidParameter(double ia, double ib)
      {
        if (ib > 1000000) // ib 是短半轴
        {
          a = ia;
          b = ib;

          f = (a - b) / a;
          e2 = (a * a - b * b) / (a * a);
          ep2 = (a * a - b * b) / (b * b);
        }
        else if (ib < 1) // ib 是椭球第一偏心率的平方
        {
          a = ia;
          e2 = ib;

          b = sqrt(a * a * (1 - e2));
          ep2 = (a * a - b * b) / (b * b);
          f = (a - b) / a;
        }

        c = a / (1 - f);
        double m0, m2, m4, m6, m8;
        m0 = a * (1 - e2);
        m2 = 1.5 * e2 * m0;
        m4 = 1.25 * e2 * m2;
        m6 = 7 * e2 * m4 / 6;
        m8 = 9 * e2 * m6 / 8;
        a0 = m0 + m2 / 2 + 3 * m4 / 8 + 5 * m6 / 16 + 35 * m8 / 128;
        a2 = m2 / 2 + m4 / 2 + 15 * m6 / 32 + 7 * m8 / 16;
        a4 = m4 / 8 + 3 * m6 / 16 + 7 * m8 / 32;
        a6 = m6 / 32 + m8 / 16;
      }
    };

    class Rtk
    {
    public:
      struct UTMCoor
      {
        double x;
        double y;
      };
      struct WGS84Corr
      {
        double lat;
        double log;
      };

    public:
      Rtk()
          : projType('u'), meridianLine(117), utm_zone_(-10000), is_loc_north_(false), pj_utm_(nullptr)
      {
        const char *wgs84 = "+proj=longlat +datum=WGS84 +no_defs "; //GPS所用坐标系,EPSG:4326
        pj_wgs84_ = pj_init_plus(wgs84);
      }
      virtual ~Rtk()
      {
        if (pj_utm_ != NULL)
        {
          pj_free(pj_utm_);
          pj_utm_ = NULL;
        }
        if (pj_wgs84_ != NULL)
        {
          pj_free(pj_wgs84_);
          pj_wgs84_ = NULL;
        }
      }
      virtual void Parse(const void *const chass_stat, void *const pose) = 0;

    public:
      constexpr static const double PI = 3.14159265358979;
      /* Ellipsoid model constants (actual values here are for WGS84) */
      constexpr static const double SM_A = 6378137.0;
      constexpr static const double SM_B = 6356752.314;
      constexpr static const double SM_EccSquared = 6.69437999013e-03;
      constexpr static const double UTMScaleFactor = 0.9996;

    protected:
      bool wgs84ToUTM(double lat, double lon, double &x, double &y);
      bool UTMTowgs84(double x, double y, double &lat, double &lon);

      bool XY2BL(double x, double y, double &lat, double &lon);
      bool BL2XY(double lat, double lon, double &x, double &y);
      bool XYZ2BLH(double x, double y, double z, double &lat, double &lon, double &ht);
      bool BLH2XYZ(double lat, double lon, double ht, double &x, double &y, double &z);

      inline double DegToRad(double deg)
      {
        return (deg / 180.0 * PI);
      }
      inline double RadToDeg(double rad)
      {
        return (rad / PI * 180.0);
      }

      /* UTMCentralMeridian
        *
        * Determines the central meridian for the given UTM zone.
        *
        * Inputs:
        *     zone - An integer value designating the UTM zone, range [1,60].
        *
        * Returns:
        *   The central meridian for the given UTM zone, in radians, or zero
        *   if the UTM zone parameter is outside the range [1,60].
        *   Range of the central meridian is the radian equivalent of [-177,+177].
        *
        */
      inline double UTMCentralMeridian(int zone)
      {
        return DegToRad(-183.0 + (zone * 6.0));
      }
      /* ArcLengthOfMeridian
        *
        * Computes the ellipsoidal distance from the equator to a point at a
        * given latitude.
        *
        * Reference: Hoffmann-Wellenhof, B., Lichtenegger, H., and Collins, J.,
        * GPS: Theory and Practice, 3rd ed.  New York: Springer-Verlag Wien, 1994.
        *
        * Inputs:
        *     phi - Latitude of the point, in radians.
        *
        * Globals:
        *     sm_a - Ellipsoid model major axis.
        *     sm_b - Ellipsoid model minor axis.
        *
        * Returns:
        *     The ellipsoidal distance of the point from the equator, in meters.
        *
        */
      double ArcLengthOfMeridian(double phi);

      /* FootpointLatitude
        *
        * Computes the footpoint latitude for use in converting transverse
        * Mercator coordinates to ellipsoidal coordinates.
        *
        * Reference: Hoffmann-Wellenhof, B., Lichtenegger, H., and Collins, J.,
        *   GPS: Theory and Practice, 3rd ed.  New York: Springer-Verlag Wien, 1994.
        *
        * Inputs:
        *   y - The UTM northing coordinate, in meters.
        *
        * Returns:
        *   The footpoint latitude, in radians.
        *
        */
      double FootpointLatitude(double y);

      /* MapLatLonToXY
        *
        * Converts a latitude/longitude pair to x and y coordinates in the
        * Transverse Mercator projection.  Note that Transverse Mercator is not
        * the same as UTM; a scale factor is required to convert between them.
        *
        * Reference: Hoffmann-Wellenhof, B., Lichtenegger, H., and Collins, J.,
        * GPS: Theory and Practice, 3rd ed.  New York: Springer-Verlag Wien, 1994.
        *
        * Inputs:
        *    phi - Latitude of the point, in radians.
        *    lambda - Longitude of the point, in radians.
        *    lambda0 - Longitude of the central meridian to be used, in radians.
        *
        * Outputs:
        *    xy - A 2-element array containing the x and y coordinates
        *         of the computed point.
        *
        * Returns:
        *    The function does not return a value.
        *
        */
      void MapLatLonToXY(double phi, double lambda, double lambda0, UTMCoor &xy);

      /* MapXYToLatLon
        *
        * Converts x and y coordinates in the Transverse Mercator projection to
        * a latitude/longitude pair.  Note that Transverse Mercator is not
        * the same as UTM; a scale factor is required to convert between them.
        *
        * Reference: Hoffmann-Wellenhof, B., Lichtenegger, H., and Collins, J.,
        *   GPS: Theory and Practice, 3rd ed.  New York: Springer-Verlag Wien, 1994.
        *
        * Inputs:
        *   x - The easting of the point, in meters.
        *   y - The northing of the point, in meters.
        *   lambda0 - Longitude of the central meridian to be used, in radians.
        *
        * Outputs:
        *   philambda - A 2-element containing the latitude and longitude
        *               in radians.
        *
        * Returns:
        *   The function does not return a value.
        *
        * Remarks:
        *   The local variables Nf, nuf2, tf, and tf2 serve the same purpose as
        *   N, nu2, t, and t2 in MapLatLonToXY, but they are computed with respect
        *   to the footpoint latitude phif.
        *
        *   x1frac, x2frac, x2poly, x3poly, etc. are to enhance readability and
        *   to optimize computations.
        *
        */
      void MapXYToLatLon(double x, double y, double lambda0, WGS84Corr &philambda);

      /* LatLonToUTMXY
        *
        * Converts a latitude/longitude pair to x and y coordinates in the
        * Universal Transverse Mercator projection.
        *
        * Inputs:
        *   lat - Latitude of the point, in radians.
        *   lon - Longitude of the point, in radians.
        *   zone - UTM zone to be used for calculating values for x and y.
        *          If zone is less than 1 or greater than 60, the routine
        *          will determine the appropriate zone from the value of lon.
        *
        * Outputs:
        *   xy - A 2-element array where the UTM x and y values will be stored.
        *
        * Returns:
        *   void
        *
        */
      void LatLonToUTMXY(double lat, double lon, int zone, UTMCoor &xy);

      /* UTMXYToLatLon
        *
        * Converts x and y coordinates in the Universal Transverse Mercator
        * projection to a latitude/longitude pair.
        *
        * Inputs:
        *    x - The easting of the point, in meters.
        *    y - The northing of the point, in meters.
        *    zone - The UTM zone in which the point lies.
        *    southhemi - True if the point is in the southern hemisphere;
        *               false otherwise.
        *
        * Outputs:
        *    latlon - A 2-element array containing the latitude and
        *            longitude of the point, in radians.
        *
        * Returns:
        *    The function does not return a value.
        *
        */
      void UTMXYToLatLon(double x, double y, int zone, bool southhemi, WGS84Corr &latlon);

      Eigen::Quaterniond euler2Quaternion(const double roll_rad, const double pitch_rad, const double yaw_rad);
      Eigen::Vector3d Quaterniond2Euler(const double x, const double y, const double z, const double w);
      Eigen::Matrix3d Quaternion2RotationMatrix(const double x, const double y, const double z, const double w);
      Eigen::Quaterniond rotationMatrix2Quaterniond(Eigen::Matrix3d R);
      Eigen::Matrix3d euler2RotationMatrix(const double roll_rad, const double pitch_rad, const double yaw_rad);
      Eigen::Vector3d RotationMatrix2euler(Eigen::Matrix3d R);

    private:
      mutable std::mutex mutex_;
      EllipsoidParameter ellipPmt;
      double meridianLine;
      char projType; // 'u': utm, 'g': gauss-kruger

      projPJ pj_wgs84_; //球体:wgs84、投影:经纬度投影
      projPJ pj_utm_;   //球体:wgs84、投影UTM
      int utm_zone_;
      bool is_loc_north_;
    };
  } // namespace location
} // namespace nirvana