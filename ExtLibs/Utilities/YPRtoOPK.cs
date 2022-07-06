using System;
using System.Collections.Generic;
using System.Text;

namespace MissionPlanner.Utilities
{
    public class YPRtoOPK
    {
        //https://s3.amazonaws.com/mics.pix4d.com/KB/documents/New+Calibration+and+Computing+Method+for+Direct+Georeferencing+of+Image+and+Scanner+Data+Using.pdf
        //https://s3.amazonaws.com/mics.pix4d.com/KB/documents/Pix4D_Yaw_Pitch_Roll_Omega_to_Phi_Kappa_angles_and_conversion.pdf
        //https://github.com/nasa/georef_geocamutilweb/blob/master/geocamUtil/registration.py#L62
        //https://github.com/davdmaccartney/rpy_opk/blob/master/rpy_opk.py
        public static (double phi, double omega, double kappa) Convert(double roll, double pitch, double yaw)
        {
            Matrix3 yprMatrix = new Matrix3();

            yprMatrix.from_euler(roll * MathHelper.deg2rad, pitch * MathHelper.deg2rad, yaw * MathHelper.deg2rad);

            Matrix3 CBb = new Matrix3(0, 1, 0, 1, 0, 0, 0, 0, -1);

            Matrix3 CEn = new Matrix3(0,0,0,0,0,0,0,0,-1);

            Matrix3 CEB = /*CEn **/ yprMatrix * CBb;


            var vec = CEB.to_euler_yxz() * MathHelper.rad2deg;
            //return (vec.X * MathHelper.rad2deg, vec.Y * MathHelper.rad2deg, vec.Z * MathHelper.rad2deg);

            var vec2 =  (Math.Asin(CEB[0,2]) * MathHelper.rad2deg, Math.Atan2(-CEB[1, 2] , CEB[2,2]) * MathHelper.rad2deg, Math.Atan2(-CEB[0, 1], CEB[0,0]) * MathHelper.rad2deg);

            var phi = 0.0;
            var omega = Math.Sinh(-yprMatrix.item(2, 0));
            var kappa = 0.0;

            if (Math.Abs(omega - (Math.PI / 2.0)) < 0.0000001)
                kappa = Math.Atan2(yprMatrix.item(1, 2), yprMatrix.item(0, 2));
            if (Math.Abs(omega + (Math.PI / 2.0)) < 0.0000001)
                kappa = Math.Atan2(-yprMatrix.item(1, 2), -yprMatrix.item(0, 2));
            else
            {
                phi = Math.Atan2(yprMatrix.item(2, 1), yprMatrix.item(2, 2));
                kappa = Math.Atan2(yprMatrix.item(1, 0), yprMatrix.item(0, 0));
            }

            var vec3 =  (phi * MathHelper.rad2deg, omega * MathHelper.rad2deg, kappa * MathHelper.rad2deg);

            return vec2;
        }
    }
}
