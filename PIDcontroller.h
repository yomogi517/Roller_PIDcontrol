#ifndef PIDcontroller_H
#define PIDcontroller_H
 
 #include "mbed.h"
 
 class PID {
 
 public:
 
    /**
    * Kp 比例ゲイン
    * Ki 積分ゲイン
    * Kd 微分ゲイン
    * tSample 制御周期
    */
    PID(float Kp, float Ki, float Kd, float tSample);
 
    /**
    * 入力値の範囲を設定
    *
    * InMin 入力の最小値 --> 0% 
    * InMax 入力の最大値 --> 100%
    */
    void setInputLimits(float inMin, float inMax);
 
    /**
    * 出力値の範囲を設定
    *
    * InMin 出力の最小値 --> 0% 
    * InMax 出力の最大値 --> 100%
    */
    void setOutputLimits(float outMin, float outMax);
 
    void setSetPoint(float sp); //目標の設定
 
    void setProcessValue(float pv); //現在値の設定
    
    void setGain(float Kp, float Ki, float Kd); //PIDゲインの設定
 
    void setBias(float Bias); //フィードフォワード制御有効　バイアス値の設定
 
    void setIncompleteDifferential(float u); //不完全微分(実用微分)有効,不完全微分係数の設定
 
    float compute(void); //PIDの計算　戻り値はoutMinからoutMaxの範囲
    float scaledParcent(float value);
 
    float calcIncompleteDifferential(void);
 
 private:
    bool usingFeedForward; //フィードフォワード制御有効フラグ
    bool usingIncompleteDifferential; //不完全微分有効フラグ
 
    //PIDゲイン
    float Kp_, Ki_, Kd_;
 
    float setPoint_; //目標値
 
    float     controllerOutput_; //出力値(0.0~1.0)
    float prevControllerOutput_; //前回の出力値(0.0~1.0)
 
    float  inMin_,  inMax_,  inSpan_; //入力の最小値,最大値,範囲
    float outMin_, outMax_, outSpan_; //出力の最小値,最大値,範囲
 
    float  accError_; //偏差の積分値
 
    float     Error_; //現在の偏差
    float prevError_; //前回の偏差
    
    float tSample_; //制御周期
 
    float Bias_; //フィードフォワード制御バイアス値
 
    float processVariable_; //入力値
 
    float prevDiffOut; //前回の微分項の出力
 
    float u_; //不完全微分係数
 
 };
 
 #endif /* PIDcontroller_H */
 
            