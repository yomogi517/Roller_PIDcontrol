#include "PIDcontroller.h"
 
 PID::PID(float Kp, float Ki, float Kd, float tSample){
 
    tSample_ = tSample;
 
    Kp_ = Kp; Ki_ = Ki; Kd_ = Kd;
    
    setPoint_             = 0.0;
    processVariable_      = 0.0;
    controllerOutput_     = 0.0;
    prevControllerOutput_ = 0.0;
 
    accError_ = 0.0;
    Error_    = 0.0;
    prevError_= 0.0;
    Bias_     = 0.0;
 
    usingFeedForward = 0;
    usingIncompleteDifferential = 0;
 
    prevDiffOut = 0.0;
    u_ = 0.0;
 }
 
 void PID::setInputLimits(float inMin, float inMax){
 
    if(inMin >= inMax) return;
     
    inMin_  = inMin;
    inMax_  = inMax;
    inSpan_ = inMax - inMin;
 }
 
 void PID::setOutputLimits(float outMin, float outMax){
 
    if(outMin >= outMax) return;
 
    outMin_  = outMin;
    outMax_  = outMax;
    outSpan_ = outMax - outMin;
 }
 
 void PID::setBias(float Bias) {
    Bias_ = Bias;
    usingFeedForward = 1;
 }
 
 void PID::setIncompleteDifferential(float u) {
    u_ = u;
    usingIncompleteDifferential = 1;
 }
 
 void PID::setSetPoint(float sp){
    setPoint_ = sp;
 }
 
 void PID::setProcessValue(float pv){
    processVariable_ = pv;
 }
 
void PID::setGain(float Kp, float Ki, float Kd) {
    Kp_ = Kp; Ki_ = Ki; Kd_ = Kd;
}
 
 float PID::compute(){
 
    //現在値と目標値の値を0~100%の範囲に置き換える
    float scaledPV = scaledParcent((processVariable_ - inMin_) / inSpan_);
 
    float scaledSP = scaledParcent((setPoint_ - inMin_) / inSpan_);
 
    //偏差の計算
    Error_= scaledSP - scaledPV;
 
    //アンチワインドアップ
    if (!(prevControllerOutput_ >= 1 && Error_ > 0) && !(prevControllerOutput_ <= 0 && Error_ < 0)) {
        accError_ += (Error_ + prevError_) / 2.0 * tSample_; //偏差の積分値の計算
    }
 
    //偏差の微分値の計算(不完全微分が有効な場合,偏差の不完全微分値を計算)
    float diffError = usingIncompleteDifferential ? calcIncompleteDifferential() : (Error_ - prevError_) / tSample_;
    
    //フィードフォワード制御が有効な場合,バイアス値の計算
    float scaledBias = usingFeedForward ? (Bias_ - outMin_) / outSpan_ : 0.0;
 
    //PIDの計算
    controllerOutput_ = scaledParcent(scaledBias + Kp_ * Error_ + Ki_ * accError_ + Kd_ * diffError); 
 
    //出力の値,偏差の値を更新
    prevControllerOutput_ = controllerOutput_;
    prevError_ = Error_;
    //微分項の値を更新
    prevDiffOut = Kd_ * diffError;
    //PIDの出力を実際の値に変換して返す
    return ((controllerOutput_ * outSpan_) + outMin_);
 
 }
 
 float PID::scaledParcent(float value) {
 
    if(value > 1.0) {
        return 1.0;
    } else if(value < 0.0) {
        return 0.0;
    }
    else return value;
 }
 
 /**
 * 不完全微分の式
 *
 * 伝達関数
 * Yn = (Td*s / (1 + k*Td*s)) * E
 *
 * 差分の式
 * yn = (k*Td / (Δt + k*Td)) * yn-1 + (Td / (Δt + k*Td))*(en - en-1)
 *
 * Δt   制御周期
 * k    定数
 * Td   微分ゲイン
 * yn   現在の出力
 * yn-1 前回の出力
 * en   現在の偏差
 * en-1 前回の偏差
 * 
 * 参考文献 http://www.nikko-pb.co.jp/products/k_data/P12_13.pdf
 */
 
 float PID::calcIncompleteDifferential(void) {
 
    float k = 1 / (tSample_ + u_ * Kd_);
 
    return (k * u_ * prevDiffOut) + (k * (Error_ - prevError_));
 }
 