#include "IMU.h"

namespace ORB_SLAM2 {
    std::ostream& operator<<(std::ostream& os, const ImuMeasure& imu)
    {
        os << "a: " << "[" << imu._a.x << "," << imu._a.y << "," << imu._a.z << "]" << std::endl;
        os << "w: " << "[" << imu._w.x << "," << imu._w.y << "," << imu._w.z << "]" << std::endl;
        os << "timestamp: " << imu._ts << std::endl;
        return os;
    }

    Calib::Calib() {
        Tcb = cv::Mat::eye(4, 4, CV_64F);
        Tbc = cv::Mat::eye(4, 4, CV_64F);
        Cov = cv::Mat::eye(6, 6, CV_64F);
        CovWalk = cv::Mat::eye(6, 6, CV_64F);
    }

    Calib::Calib(const Calib &calib) {
        Tcb = calib.Tcb.clone();
        Tbc = calib.Tbc.clone();
        Cov = calib.Cov.clone();
        CovWalk = calib.CovWalk.clone();
    }

    void Calib::setCalib(const cv::Mat &Tbc_, const float &ng, const float &na, const float &ngw, const float &naw) {
        // Deal with Tbc later
        Tbc = Tbc_.clone();
        Tcb = cv::Mat::eye(4,4,CV_64F);
        
        // Simple matrix inversion
        // g inv = [R', -R't; 0, 1]
        Tcb.rowRange(0,3).colRange(0,3) = Tbc.rowRange(0,3).colRange(0,3).t();
        Tcb.rowRange(0,3).col(3) = -Tbc.rowRange(0,3).colRange(0,3).t() * Tbc.rowRange(0,3).col(3);

        const float ng2 = ng * ng;
        const float na2 = na * na;
        Cov = cv::Mat::eye(6,6,CV_64F);
        Cov.rowRange(0,3).colRange(0,3) *= ng2;
        Cov.rowRange(3,6).colRange(3,6) *= na2;

        CovWalk = cv::Mat::eye(6,6,CV_64F);
        const float ngw2 = ngw * ngw;
        const float naw2 = naw * naw;
        CovWalk.rowRange(0,3).colRange(0,3) *= ngw2;
        CovWalk.rowRange(3,6).colRange(3,6) *= naw2;
    }

    void PreintegratedIMU::initialize() {
        Rwb = cv::Mat::eye(3, 3,   CV_64F);
        wVb = cv::Mat::zeros(3, 1, CV_64F);
        wPb = cv::Mat::zeros(3, 1, CV_64F);

        Rwl = cv::Mat::eye(3, 3,   CV_64F);
        wVl = cv::Mat::zeros(3, 1, CV_64F);
        wPl = cv::Mat::zeros(3, 1, CV_64F);

        nphi = cv::Mat::zeros(3, 1, CV_64F);
        nv = cv::Mat::zeros(3, 1, CV_64F);
        np = cv::Mat::zeros(3, 1, CV_64F);
        JRg = cv::Mat::zeros(3, 3, CV_64F);
        Jva = cv::Mat::zeros(3, 3, CV_64F);
        Jvg = cv::Mat::zeros(3, 3, CV_64F);
        Jpa = cv::Mat::zeros(3, 3, CV_64F);
        Jpg = cv::Mat::zeros(3, 3, CV_64F);

        Rlc = cv::Mat::eye(3, 3, CV_64F);
        lPc = cv::Mat::zeros(3, 1, CV_64F);
        lVc = cv::Mat::zeros(3, 1, CV_64F);
        lastTimestamp = 0;
        currentTimestamp = 0;
    }

    void PreintegratedIMU::mergePrior(const PreintegratedIMU& prev) {
        Rwb = prev.Rwb.clone();
        wVb = prev.wVb.clone();
        wPb = prev.wPb.clone();

        // Those are const
        Rwl = prev.Rwb.clone();
        wVl = prev.wVb.clone();
        wPl = prev.wPb.clone();

        // nphi = prev.nphi.clone();
        // nv = prev.nv.clone();
        // np = prev.np.clone();
        // JRg = prev.JRg.clone();
        // Jva = prev.Jva.clone();
        // Jvg = prev.Jvg.clone();
        // Jpa = prev.Jpa.clone();
        // Jpg = prev.Jpg.clone();

        // Even noises are propagated, they are not propagated here. 
        nphi = cv::Mat::zeros(3, 1, CV_64F);
        nv = cv::Mat::zeros(3, 1, CV_64F);
        np = cv::Mat::zeros(3, 1, CV_64F);
        // Even if we update bias, it is only within those measurements
        JRg = cv::Mat::zeros(3, 3, CV_64F);
        Jva = cv::Mat::zeros(3, 3, CV_64F);
        Jvg = cv::Mat::zeros(3, 3, CV_64F);
        Jpa = cv::Mat::zeros(3, 3, CV_64F);
        Jpg = cv::Mat::zeros(3, 3, CV_64F);

        // This is within thie particular factor
        Rlc = cv::Mat::eye(3, 3, CV_64F);
        lPc = cv::Mat::zeros(3, 1, CV_64F);
        lVc = cv::Mat::zeros(3, 1, CV_64F);

        
        b = prev.b;
        c = prev.c;
        lastTimestamp = prev.currentTimestamp;
    }

    std::ostream& operator<<(std::ostream& os, const PreintegratedIMU& imu)
    {
        os << imu.currentTimestamp << "," << imu.wPb.at<double>(0, 0) << "," << imu.wPb.at<double>(1, 0) << "," << imu.wPb.at<double>(2, 0) << ",";
        auto quat = imu.convertRotm2Quat(imu.Rwb); // x, y, z, w
        os << quat.at<double>(3, 0);
        for (int i = 0; i < 3; i++) {
            os << "," << quat.at<double>(i, 0);
        }
        return os;
    }

    void PreintegratedIMU::integrateMeasurement(const cv::Point3f a, const cv::Point3f w, float dt) {
        // Integrate Measurement
        cv::Mat acc = cv::Mat::zeros(3, 1, CV_64F); // 5 : cv::CV_32F, no cv:: required
        acc.at<double>(0, 0) = a.x - b.bax;
        acc.at<double>(1, 0) = a.y - b.bay;
        acc.at<double>(2, 0) = a.z - b.baz; // - 1.226e-4;
        
        cv::Mat angV = cv::Mat::zeros(3, 1, CV_64F);
        angV.at<double>(0, 0) = w.x - b.bwx;
        angV.at<double>(1, 0) = w.y - b.bwy;
        angV.at<double>(2, 0) = w.z - b.bwz;

        // These needs some verification
        auto TcbR = c.Tcb.rowRange(0, 3).colRange(0, 3);
        acc = TcbR * acc;
        angV = TcbR * angV;
        
        // Equation 2
        wPb = wPb + wVb * dt + 0.5 * g_w * pow(dt, 2) + 0.5 * Rwb * acc * pow(dt, 2);
        wVb = wVb + g_w * dt + Rwb * acc * dt;
        auto dR = ExpMap(angV, dt);
        Rwb = Rwb * dR;
        Rlc = Rlc * dR;

        // std::cout << "After Equation 2" << std::endl;

        // Update Covariance
        cv::Mat gyro_noise = c.Cov.rowRange(0, 3).colRange(0, 3).clone();
        cv::Mat acc_noise = c.Cov.rowRange(3, 6).colRange(3, 6).clone();
        np = np + nv * dt
            - 0.5 * Rlc * hatOperator(acc) * nphi * dt*dt;
            + 0.5 * Rlc * acc_noise.diag(0) * dt*dt;
        std::cout << np;
        
        nv = nv - Rlc * hatOperator(acc) * nphi * dt + Rlc * acc_noise.diag(0) * dt;
        nphi = dR.t() * nphi + rightJacobian(angV) * gyro_noise.diag(0) * dt;
        // Retrieve Covariance matrix through getMeasurementNoise
        
        // Update Jacobian
        Jpa = Jpa + 1.5 * Jva * dt;
        Jpg = Jpg + 1.5 * Jvg * dt;
        Jva = Jva - Rlc * dt;
        Jvg = Jvg - Rlc * hatOperator(acc) * JRg * dt;
        JRg = dR.t() * JRg - dR.t() * rightJacobian(angV) * dt;

        // Update Timestamp in case people forget.
        currentTimestamp += dt;
    }

    cv::Mat PreintegratedIMU::ExpMap(cv::Mat angV, float t) {
        // This angV has bias taken away
        cv::Mat I = cv::Mat::eye(3, 3, CV_64F);
        cv::Mat w = angV * t;

        cv::Mat w_hat = hatOperator(w);

        cv::Mat R = I + w_hat * sin(t) + w_hat * w_hat * (1 - cos(t));
        return R;
    }

    cv::Mat hatOperator(cv::Mat w) {
        cv::Mat w_hat = (cv::Mat_<double>(3,3)
            << 0, -w.at<double>(2, 0), w.at<double>(1, 0), 
               w.at<double>(2, 0), 0, -w.at<double>(0, 0),
              -w.at<double>(1, 0), w.at<double>(0, 0), 0);
        return w_hat.clone();
    }

    cv::Mat rightJacobian(cv::Mat phi) {
        cv::Mat I = cv::Mat::eye(3, 3, CV_64F);
        double phi_norm = cv::norm(phi);

        cv::Mat phi_hat = hatOperator(phi);
        
        auto rJ = I - (1 - cos(phi_norm)) / (phi_norm * phi_norm) * phi_hat
            + (phi_norm) - sin(phi_norm) / (pow(phi_norm, 3)) * phi_hat * phi_hat;
        return rJ;
    }

    void PreintegratedIMU::updateBias(const Bias& b_) {
        cv::Mat dBg = (cv::Mat_<double>(3, 1)
                << b_.bwx - b.bwx, b_.bwy - b.bwy, b_.bwz - b.bwz);

        cv::Mat dBa = (cv::Mat_<double>(3, 1)
                << b_.bax - b.bax, b_.bay - b.bay, b_.baz - b.baz);

        // Rlc before and after the following function should be
        // exactly the same
        updateDeltaTransformation();
        Rlc *= ExpMap(JRg * dBg, 1.);
        lVc += Jvg * dBg + Jva * dBa;
        lPc += Jpg * dBg + Jpa * dBa;
        
        // Then update Rwb wVb and wPb
        double dt = currentTimestamp - lastTimestamp;
        // Rlc = Rwl.t() * Rwb;
        Rwb = Rwl * Rlc;
        // lVc = Rwl.t() * (wVb - wVl - g_w * (currentTimestamp - lastTimestamp));
        wVb = Rwl * lVc + wVl + g_w * dt;
        // lPc = Rwl.t() * (wPb - wPl - wVl * dt - 0.5 * g_w * pow(dt, 2));
        wPb = Rwl * lPc + wPl + wVl * dt + 0.5 * g_w * dt*dt;
    }

    cv::Mat PreintegratedIMU::getMeasurementNoise() {
        cv::Mat covarianceMatrix = cv::Mat::zeros(9, 9, CV_64F);
        covarianceMatrix.rowRange(0, 3).colRange(0, 3) *= nphi;
        covarianceMatrix.rowRange(3, 6).colRange(3, 6) *= nv;
        covarianceMatrix.rowRange(6, 9).colRange(6, 9) *= np;
        return covarianceMatrix.clone();
    }

    cv::Mat PreintegratedIMU::getRotation() {
        return Rwb;
    }

    cv::Mat PreintegratedIMU::getTranslation() {
        return wPb;
    }

    double PreintegratedIMU::getTS() {
        return currentTimestamp;
    }

    cv::Mat PreintegratedIMU::convertRotm2Quat(const cv::Mat R) const
    {
        cv::Mat result = cv::Mat(4, 1, CV_64F);
        double trace = R.at<double>(0,0) + R.at<double>(1,1) + R.at<double>(2,2);
    
        if (trace > 0.0) 
        {
            double s = sqrt(trace + 1.0);
            result.at<double>(3, 0) = (s * 0.5);
            s = 0.5 / s;
            result.at<double>(0, 0) = ((R.at<double>(2,1) - R.at<double>(1,2)) * s);
            result.at<double>(1, 0) = ((R.at<double>(0,2) - R.at<double>(2,0)) * s);
            result.at<double>(2, 0) = ((R.at<double>(1,0) - R.at<double>(0,1)) * s);
        } 
        
        else 
        {
            int i = R.at<double>(0,0) < R.at<double>(1,1) ? (R.at<double>(1,1) < R.at<double>(2,2) ? 2 : 1) : (R.at<double>(0,0) < R.at<double>(2,2) ? 2 : 0); 
            int j = (i + 1) % 3;  
            int k = (i + 2) % 3;

            double s = sqrt(R.at<double>(i, i) - R.at<double>(j,j) - R.at<double>(k,k) + 1.0);
            result.at<double>(i, 0) = s * 0.5;
            s = 0.5 / s;

            result.at<double>(3, 0) = (R.at<double>(k,j) - R.at<double>(j,k)) * s;
            result.at<double>(j, 0) = (R.at<double>(j,i) + R.at<double>(i,j)) * s;
            result.at<double>(k, 0) = (R.at<double>(k,i) + R.at<double>(i,k)) * s;
        }
        return result;
    }

    void PreintegratedIMU::setInitialState(cv::Mat R_, cv::Mat p, cv::Mat v, float ts) {
        Rwl = R_.clone();
        wPl = p.clone();
        wVl = v.clone();
        lastTimestamp = ts;
    }

    cv::Mat PreintegratedIMU::getDeltaR()
    {
        updateDeltaTransformation();
        return Rlc;
    }

    cv::Mat PreintegratedIMU::getDeltaP()
    {
        updateDeltaTransformation();
        return lPc;
    }

    cv::Mat PreintegratedIMU::getDeltaV()
    {
        updateDeltaTransformation();
        return lVc;
    }

    void PreintegratedIMU::updateDeltaTransformation() {
        float dt = currentTimestamp - lastTimestamp;
        Rlc = Rwl.t() * Rwb;
        lVc = Rwl.t() * (wVb - wVl - g_w * (currentTimestamp - lastTimestamp));
        lPc = Rwl.t() * (wPb - wPl - wVl * dt - 0.5 * g_w * pow(dt, 2));
    }


}