/*!
 * \file Law.h
 * \brief Class defining a PID
 * \author Guillaume Sanahuja, Copyright Heudiasyc UMR UTC/CNRS 7253
 * \date 2011/05/01
 * \version 4.0
 */

#ifndef ATTITUDECONTROL_H
#define ATTITUDECONTROL_H

#include <ControlLaw.h>

namespace flair {
  namespace core {
    class AhrsData;
  }
  namespace gui {
    class DoubleSpinBox;
  }
}

namespace flair { namespace filter {
  /*! \class AttitudeControl
  *
  * \brief Class defining a PID
  */
  class AttitudeControl : public ControlLaw {
    public:
      /*!
      * \brief Constructor
      *
      * Construct a PID at given position. \n
      * The PID will automatically be child of position->getLayout() Layout. After calling this function,
      * position will be deleted as it is no longer usefull. \n
      *
      * \param position position to display settings
      * \param name name
      */
      AttitudeControl(const gui::LayoutPosition* position,std::string name);

      /*!
      * \brief Destructor
      *
      */
      ~AttitudeControl();

      /*!
      * \brief Reset integral
      *
      */
      void Reset(void);

      /*!
      * \brief Set input values
      *
      * \param actualOrientation actual orientation
      * \param referenceOrientation reference orientation
      */
      void SetValues(const core::AhrsData *actualOrientation,const core::AhrsData *referenceOrientation);

      /*!
      * \brief Use default plot
      *
      * Plot the output values at position. \n
      * Plot consists of 4 curves: proportional part,
      * derivative part, integral part and
      * the sum of the three. \n
      * After calling this function, position will be deleted as it is no longer usefull. \n
      *
      * \param position position to display plot
      */
      void UseDefaultPlot(const gui::LayoutPosition* position);
     
    private:
      
      /*!
      * \brief Update using provided datas
      *
      * Reimplemented from IODevice.
      *
      * \param data data from the parent to process
      */
      void UpdateFrom(const core::io_data *data);

      gui::DoubleSpinBox *kp1,*kd1,*kp2,*kd2,*kp3,*kd3,*alpha,*beta;
      core::Matrix *state;
  };
} // end namespace filter
} // end namespace flair
#endif //ATTITUDECONTROL_H
