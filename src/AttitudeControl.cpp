//  created:    2011/05/01
//  filename:   AttitudeControl.cpp
//
//  author:     Guillaume Sanahuja
//              Copyright Heudiasyc UMR UTC/CNRS 7253
//
//  version:    $Id: $
//
//  purpose:    
//
//
/*********************************************************************/

#include "AttitudeControl.h"
#include <Matrix.h>
#include <LayoutPosition.h>
#include <DoubleSpinBox.h>
#include <GroupBox.h>
#include <AhrsData.h>
#include <DataPlot1D.h>
#include <math.h>

using std::string;
using namespace flair::core;
using namespace flair::gui;

namespace flair { namespace filter {

AttitudeControl::AttitudeControl(const LayoutPosition* position,string name) : ControlLaw(position->getLayout(),name,3) {
  //init matrix
  input=new Matrix(this,14,1,floatType,name);
  MatrixDescriptor* desc=new MatrixDescriptor(7,1);
  desc->SetElementName(0,0,"q0e");
  desc->SetElementName(1,0,"q1e");
  desc->SetElementName(2,0,"q2e");
  desc->SetElementName(3,0,"q3e");
  desc->SetElementName(4,0,"Rdq1e");
  desc->SetElementName(5,0,"Rdq2e");
  desc->SetElementName(6,0,"Rdq3e");
  state=new Matrix(this,desc,floatType,name);

  AddDataToLog(state);

  Reset();
  
  GroupBox* groupbox=new GroupBox(position,name);
  kp1 = new DoubleSpinBox(groupbox->NewRow(),"kp1:",0,2000,100);
  kd1 = new DoubleSpinBox(groupbox->LastRowLastCol(),"kd1:",0,2000,100);
  kp2 = new DoubleSpinBox(groupbox->NewRow(),"kp2:",0,2000,100);
  kd2 = new DoubleSpinBox(groupbox->LastRowLastCol(),"kd2:",0,2000,100);
  kp3 = new DoubleSpinBox(groupbox->NewRow(),"kp3:",0,2000,100);
  kd3 = new DoubleSpinBox(groupbox->LastRowLastCol(),"kd3:",0,2000,100);
  alpha = new DoubleSpinBox(groupbox->NewRow(),"alpha:",0,2000,100);
  beta  = new DoubleSpinBox(groupbox->LastRowLastCol(),"beta:",0,2000,100);
}

AttitudeControl::~AttitudeControl(void) {
}

void AttitudeControl::UseDefaultPlot(const gui::LayoutPosition* position) {
  DataPlot1D *plot=new DataPlot1D(position,ObjectName(),-1,1);
  plot->AddCurve(output->Element(0),DataPlot::Red,"x");
  plot->AddCurve(output->Element(1),DataPlot::Green,"y");
  plot->AddCurve(output->Element(2),DataPlot::Blue,"z");
}

void AttitudeControl::Reset(void) {

}

void AttitudeControl::UpdateFrom(const io_data *data) {
  input->GetMutex();
  Quaternion actualQuaternion(input->ValueNoMutex(0,0),input->ValueNoMutex(1,0),input->ValueNoMutex(2,0),input->ValueNoMutex(3,0));
  Quaternion referenceQuaternion(input->ValueNoMutex(4,0),input->ValueNoMutex(5,0),input->ValueNoMutex(6,0),input->ValueNoMutex(7,0));
  Vector3Df actualOmega(input->ValueNoMutex(8,0),input->ValueNoMutex(9,0),input->ValueNoMutex(10,0));
  Vector3Df referenceOmega(input->ValueNoMutex(11,0),input->ValueNoMutex(12,0),input->ValueNoMutex(13,0));
  input->ReleaseMutex();
  
  Quaternion errorQuaternion = actualQuaternion*(referenceQuaternion.GetConjugate());
  Vector3Df qe;
  qe.x = errorQuaternion.q1;
  qe.y = errorQuaternion.q2;
  qe.z = errorQuaternion.q3;

  Vector3Df Omegae = actualOmega-referenceOmega;
  qe.Rotate(referenceQuaternion.GetConjugate());
  
  Vector3Df torques;
  
  torques.x=float(kd1->Value())*Omegae.x+float(kp1->Value())*qe.x;
  torques.y=float(kd2->Value())*Omegae.y+float(kp2->Value())*qe.y;
  torques.z=float(kd3->Value())*Omegae.z+float(kp3->Value())*qe.z;

  torques.x+=beta->Value()*tanh(alpha->Value()*torques.x);
  torques.y+=beta->Value()*tanh(alpha->Value()*torques.y);
  torques.z+=beta->Value()*tanh(alpha->Value()*torques.z);

  state->GetMutex();
  state->SetValueNoMutex(0,0,errorQuaternion.q0);
  state->SetValueNoMutex(1,0,errorQuaternion.q1);
  state->SetValueNoMutex(2,0,errorQuaternion.q2);
  state->SetValueNoMutex(3,0,errorQuaternion.q3);
  state->SetValueNoMutex(4,0,qe.x);
  state->SetValueNoMutex(5,0,qe.y);
  state->SetValueNoMutex(6,0,qe.z);
  state->ReleaseMutex();
    
  output->SetValue(0,0,torques.x);
  output->SetValue(1,0,torques.y);
  output->SetValue(2,0,torques.z);
  output->SetDataTime(data->DataTime());

  ProcessUpdate(output);
}

void AttitudeControl::SetValues(const AhrsData *actualOrientation,const AhrsData *referenceOrientation) {
  Quaternion actualQuaternion,referenceQuaternion;
  Vector3Df actualOmega,referenceOmega;
  actualOrientation->GetQuaternionAndAngularRates(actualQuaternion,actualOmega);
  referenceOrientation->GetQuaternionAndAngularRates(referenceQuaternion,referenceOmega);

  input->GetMutex();
  input->SetValue(0,0,actualQuaternion.q0);
  input->SetValue(1,0,actualQuaternion.q1);
  input->SetValue(2,0,actualQuaternion.q2);
  input->SetValue(3,0,actualQuaternion.q3);
  input->SetValue(4,0,referenceQuaternion.q0);
  input->SetValue(5,0,referenceQuaternion.q1);
  input->SetValue(6,0,referenceQuaternion.q2);
  input->SetValue(7,0,referenceQuaternion.q3);
  input->SetValue(8,0,actualOmega.x);
  input->SetValue(9,0,actualOmega.y);
  input->SetValue(10,0,actualOmega.z);
  input->SetValue(11,0,referenceOmega.x);
  input->SetValue(12,0,referenceOmega.y);
  input->SetValue(13,0,referenceOmega.z);
  input->ReleaseMutex();
}

} // end namespace filter
} // end namespace flair
