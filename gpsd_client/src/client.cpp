#include <ros/ros.h>
#include <gps_common/GPSFix.h>
#include <gps_common/GPSStatus.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <libgpsmm.h>

using namespace gps_common;
using namespace sensor_msgs;

class GPSDClient {
  public:
    GPSDClient() : privnode("~"), gps("localhost", DEFAULT_GPSD_PORT), use_gps_time(false), check_fix_by_variance(false) {}

    bool start() {
      gps_fix_pub = node.advertise<GPSFix>("extended_fix", 1);
      navsat_fix_pub = node.advertise<NavSatFix>("fix", 1);

      privnode.getParam("use_gps_time", use_gps_time);
      privnode.getParam("check_fix_by_variance", check_fix_by_variance);

      std::string host = "localhost";
      int port = 2947;
      privnode.getParam("host", host);
      privnode.getParam("port", port);

      char port_s[12];
      snprintf(port_s, 12, "%d", port);

      gps_data_t *resp = NULL;

#if GPSD_API_MAJOR_VERSION >= 5
      while (gps.stream(WATCH_ENABLE|WATCH_JSON) == NULL) {
        ROS_ERROR("Failed to open GPSD (is GPSD running?)");
        sleep(1);
      }

#elif GPSD_API_MAJOR_VERSION == 4
      resp = gps.stream(WATCH_ENABLE);
#elif GPSD_API_MAJOR_VERSION == 3
      gps.query("w\n");
#else
#error "gpsd_client only supports gpsd API versions >= 3"
#endif

      ROS_INFO("GPSd opened");
      return true;
    }

    void step() {
      ROS_DEBUG("Stepping");
      struct gps_data_t* p;

#if GPSD_API_MAJOR_VERSION >= 5      
      if (!gps.waiting(5000000))
        return;
      if ((p = gps.read()) == NULL) {
        ROS_ERROR_THROTTLE(1, "GPSD client read error (has GPSD shutdown?).");
        return;
      } else {
        ROS_DEBUG("Going to process data");
#else
        gps_data_t *p = gps->poll();
#endif
        process_data(p);
        ROS_DEBUG("Data processed");      
      }
    }

    void stop() {
      // gpsmm doesn't have a close method? OK ...
    }

  private:
    ros::NodeHandle node;
    ros::NodeHandle privnode;
    ros::Publisher gps_fix_pub;
    ros::Publisher navsat_fix_pub;
    gpsmm gps;

    bool use_gps_time;
    bool check_fix_by_variance;

    void process_data(struct gps_data_t* p) {
      if (p == NULL) {
        ROS_ERROR("Null data received");
        return;
      }

      if (!p->online) {
        ROS_WARN("Not online");      
        return;
      }

      ROS_DEBUG("Procesing data for old gpsfix message (with extended info) now.");
      process_data_gps(p);
      ROS_DEBUG("Procesing data for new standard navsatfix message now.");
      process_data_navsat(p);
    }


#if GPSD_API_MAJOR_VERSION >= 4
#define SATS_VISIBLE p->satellites_visible
#elif GPSD_API_MAJOR_VERSION == 3
#define SATS_VISIBLE p->satellites
#endif

    void process_data_gps(struct gps_data_t* p) {
      ros::Time time = ros::Time::now();

      GPSFix fix;
      GPSStatus status;

      status.header.stamp = time;
      fix.header.stamp = time;

      status.satellites_used = p->satellites_used;

      ROS_DEBUG("Satellites used: %d", p->satellites_used);
      status.satellite_used_prn.resize(status.satellites_used);
      for (int i = 0; i < status.satellites_used; ++i) {
        status.satellite_used_prn[i] = p->used[i];
      }

      status.satellites_visible = SATS_VISIBLE;

      status.satellite_visible_prn.resize(status.satellites_visible);
      status.satellite_visible_z.resize(status.satellites_visible);
      status.satellite_visible_azimuth.resize(status.satellites_visible);
      status.satellite_visible_snr.resize(status.satellites_visible);

      if(status.satellites_used > 0) {
        ROS_DEBUG("Satellites visible: %d", SATS_VISIBLE);
        for (int i = 0; i < SATS_VISIBLE; ++i) {
          status.satellite_visible_prn[i] = p->PRN[i];
          status.satellite_visible_z[i] = p->elevation[i];
          status.satellite_visible_azimuth[i] = p->azimuth[i];
          status.satellite_visible_snr[i] = p->ss[i];
        }
      }
      ROS_DEBUG("Latitude: %f", p->fix.latitude);
      ROS_DEBUG("Latitude: %f", p->fix.longitude);
      
      ROS_DEBUG("Status: %d", p->status);
      if ((p->status & STATUS_FIX) && !(check_fix_by_variance && isnan(p->fix.epx))) {
        ROS_DEBUG("Building fix message");
        status.status = 0; // FIXME: gpsmm puts its constants in the global
                           // namespace, so `GPSStatus::STATUS_FIX' is illegal.

        if (p->status & STATUS_DGPS_FIX)
          status.status |= 18; // same here

        fix.time = p->fix.time;
        fix.latitude = p->fix.latitude;
        fix.longitude = p->fix.longitude;
        fix.altitude = p->fix.altitude;
        fix.track = p->fix.track;
        fix.speed = p->fix.speed;
        fix.climb = p->fix.climb;

#if GPSD_API_MAJOR_VERSION > 3
        fix.pdop = p->dop.pdop;
        fix.hdop = p->dop.hdop;
        fix.vdop = p->dop.vdop;
        fix.tdop = p->dop.tdop;
        fix.gdop = p->dop.gdop;
#else
        fix.pdop = p->pdop;
        fix.hdop = p->hdop;
        fix.vdop = p->vdop;
        fix.tdop = p->tdop;
        fix.gdop = p->gdop;
#endif

        fix.err = p->epe;
        fix.err_vert = p->fix.epv;
        fix.err_track = p->fix.epd;
        fix.err_speed = p->fix.eps;
        fix.err_climb = p->fix.epc;
        fix.err_time = p->fix.ept;
        
        ROS_DEBUG("FIX_ERR values - epe: %f", p->epe );
        ROS_DEBUG("FIX_ERR values - epx: %f", p->fix.epx);
        ROS_DEBUG("FIX_ERR values - epy: %f", p->fix.epy);
        ROS_DEBUG("FIX_ERR values - epv: %f", p->fix.epv );
        ROS_DEBUG("FIX_ERR values - track: %f", p->fix.epd );
        ROS_DEBUG("FIX_ERR values - speed: %f", p->fix.eps );
        ROS_DEBUG("FIX_ERR values - climb: %f", p->fix.epc );
        ROS_DEBUG("FIX_ERR values - time: %f", p->fix.ept );
        
        
        ROS_DEBUG("GST utc: %f", p->gst.utctime);
        ROS_DEBUG("GST rms: %f", p->gst.rms_deviation);
        ROS_DEBUG("GST maj: %f", p->gst.smajor_deviation);
        ROS_DEBUG("GST min: %f", p->gst.sminor_deviation);
        ROS_DEBUG("GST ori: %f", p->gst.smajor_orientation);
        ROS_DEBUG("GST lat_standard_deviation: %f", p->gst.lat_err_deviation);
        ROS_DEBUG("GST lon_standard_deviation: %f", p->gst.lon_err_deviation);
        ROS_DEBUG("GST:alt_standard_deviation: %f", p->gst.alt_err_deviation);

        /* TODO: attitude */
      } else {
      	status.status = -1; // STATUS_NO_FIX
      }

      fix.status = status;

      gps_fix_pub.publish(fix);
    }

    void process_data_navsat(struct gps_data_t* p) {
      NavSatFixPtr fix(new NavSatFix);

      // adding frame_id so we can visualize it in RViz
      fix->header.frame_id = "world";
      /* TODO: Support SBAS and other GBAS. */

      if (use_gps_time)
        fix->header.stamp = ros::Time(p->fix.time);
      else
        fix->header.stamp = ros::Time::now();

      /* gpsmm pollutes the global namespace with STATUS_,
       * so we need to use the ROS message's integer values
       * for status.status
       */
      switch (p->status) {
        case STATUS_NO_FIX:
          fix->status.status = -1; // NavSatStatus::STATUS_NO_FIX;
          break;
        case STATUS_FIX:
          fix->status.status = 0; // NavSatStatus::STATUS_FIX;
          break;
        case STATUS_DGPS_FIX:
          fix->status.status = 2; // NavSatStatus::STATUS_GBAS_FIX;
          break;
      }

      fix->status.service = NavSatStatus::SERVICE_GPS;

      fix->latitude = p->fix.latitude;
      fix->longitude = p->fix.longitude;
      fix->altitude = p->fix.altitude;

      /* gpsd reports status=OK even when there is no current fix,
       * as long as there has been a fix previously. Throw out these
       * fake results, which have NaN variance
       */
       
      //TODO: añadir aqui la comprobación que se reporta error elíptico, y si es así convertilo a matriz de covarianza (con valores fuera de la diagonal a partir de la orientación de la elipse), que aporta aun más información
      if (isnan(p->gst.smajor_deviation))
        ROS_WARN("GPS not reporting oriented error elipse, falling back to lat-long oriented standard deviation.");  

      // PATCH: miramos primero si se reportaron valores de error en desviación estándar en m.
      
      if (!isnan(p->gst.lat_err_deviation)) { //los valores al ser covarianza deberían ser ^2
                                              //, pero no estoy muy seguro y los dejo originales pq se inestabilizan mucho
                                              
        fix->position_covariance[0] = p->gst.lat_err_deviation; // * p->gst.lat_err_deviation;
        fix->position_covariance[4] = p->gst.lon_err_deviation; // * p->gst.lon_err_deviation;
        fix->position_covariance[8] = p->gst.alt_err_deviation; // * p->gst.alt_err_deviation;

        fix->position_covariance_type = NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
      } else if (!isnan(p->fix.epx)) {
        ROS_WARN("GPS not reporting lat-long oriented standard deviation, falling back to epx, epy, epv (50-50 confidence).");
        fix->position_covariance[0] = p->fix.epx;
        fix->position_covariance[4] = p->fix.epy;
        fix->position_covariance[8] = p->fix.epv;

        fix->position_covariance_type = NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
      } else if (check_fix_by_variance) {
        ROS_WARN("GPS not reporting error values and check_fix_by_variance parameter is true, thus we will not publish this fix.");
        return;
      }
      navsat_fix_pub.publish(fix);
    }
};

int main(int argc, char ** argv) {
  ros::init(argc, argv, "gpsd_client");

  GPSDClient client;

  if (!client.start())
    return -1;


  while(ros::ok()) {
    ros::spinOnce();
    client.step();
  }

  client.stop();
}
