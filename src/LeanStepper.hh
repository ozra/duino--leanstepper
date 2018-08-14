#ifndef LEANSTEPPER_H
#define LEANSTEPPER_H

#include <stdlib.h>
#include <Arduino.h>
#include "OnyxTypeAliases.h"
#include "TerseConvenienceKeywords.h"
#include "ChronoTrigger.h"
#include "HolyDio.h"

// *TODO* - safety limits (lower / upper position)

template <typename T>
I8 sign(T x) {
   // assumes 32-bit int and 2s complement signed shifts work (implementation defined by C spec)
   // *TODO* this is totally hardcoded and misleading with the T...
   return (x >> 31) | ((U32)-x >> 31);
}

typedef I32 StepperPos;
typedef I32 StepperDelta;
typedef I8  Sign;

struct LeanStepper4Bipolar {};
struct LeanStepper47_Osv_Etc_Mm {};

/* struct LeanStepperMotionLinear {}; */
struct LeanStepperZeroInertia {};


// This is still pure virtual because of ChronoTrigger update
class LeanStepperBase : public ChronoTrigger { // ChronoTrigger<true> {
  protected:
   const Duration minimum_pulse_width_;
   const Duration pulse_width_;

   StepperPos     current_pos_ = 0;
   StepperDelta   steps_to_move_ = 0;
   Sign           direction_ = 0;

  public:
   /*/ Subclasses use only the states needed from below */
   /* static const ChronoState Still         = Default; */
   static const ChronoState StepUnderflow = 0;
   static const ChronoState Step1         = 1;
   static const ChronoState Step2         = 2;
   static const ChronoState Step3         = 3;
   static const ChronoState Step4         = 4;
   static const ChronoState Step5         = 5;
   static const ChronoState Step6         = 6;
   static const ChronoState Step7         = 7;
   static const ChronoState Step8         = 8;

   /* constexpr LeanStepperBase(const I32 Rpm, const I32 StepsPr) */
   /* : */
   /*    minimum_pulse_width_(60000000L / (Rpm * StepsPr)), */
   /*    pulse_width_(minimum_pulse_width_) */
   /* {} */

   LeanStepperBase(const I16 Rpm, const I16 StepsPr)
   :
      minimum_pulse_width_(60000000L / (I32(Rpm) * StepsPr)),
      pulse_width_(minimum_pulse_width_)
   {}

   fn is_ready() -> Bool {
      return direction_ == 0;
   }

   fn log() -> Void {
      /* say(current_pos_); */
   }

   fn moveTo(StepperPos position) -> Void {
      move(position - current_pos_);
   }

   fn move(StepperDelta delta) -> Void {
      steps_to_move_ = delta;
      direction_ = sign(delta);
   }

   // stop and hold position with power
   fn stop_hold() -> Void {
      steps_to_move_ = 0;
      direction_ = 0;
   }

   // stop and turn off all power supply
   fn stop_off() -> Void {
      stop_hold();
      power_down_();
   }

   fn reset_ref_pos(StepperPos position = 0) -> Void {
      current_pos_ = position;
   }

   fn current_pos() -> StepperPos {
      // Since we're always a step ahead while moving, we compensate by -step.
      // The "true" (fractional) position depends on how soon after last step
      // this call is made. Erring on this side gives the _most likely minimal
      // error_ compared to the state of physical reality.
      return current_pos_ - direction_;
   }

   // Aliases for AccelStepper compat for comparison tests
   fn setCurrentPosition(StepperPos position = 0) -> Void { reset_ref_pos(position); }
   fn currentPosition() -> StepperPos { return current_pos(); }
   fn stop() -> Void { stop_hold(); }

  protected:
   virtual fn power_down_() -> Void = 0;

};

// = LeanStepper4B
/* template < */
/*  typename MT, */
/*  U8 P1, U8 P2 = P1 + 1, U8 P3 = P2 + 1, U8 P4 = P3 + 1, */
/*  U16 Rpm = 150, U16 StepsPR = 200, */
/*  typename VT = LeanStepperZeroInertia */
/* > */

/* template <typename VT = LeanStepperZeroInertia> */
template <
 U8 Pin1, U8 Pin2 = Pin1 + 1, U8 Pin3 = Pin2 + 1, U8 Pin4 = Pin3 + 1,
 /* U16 Rpm = 150, U16 StepsPR = 200, */
 typename VT = LeanStepperZeroInertia
>
class LeanStepper4B : public LeanStepperBase { // <VT> {
 public:
   static const ChronoState StepOverflow4 = 5;

   LeanStepper4B(
      /* const U8 Pin1, const U8 Pin2, const U8 Pin3, const U8 Pin4, */
      const I16 Rpm, const I16 StepsPr
   ) :
      /* Pin1(Pin1), Pin2(Pin2), Pin3(Pin3), Pin4(Pin4), */
      LeanStepperBase(Rpm, StepsPr)
   {}

   fn update() -> Void {
      let where_to = where_to_go();

      if (where_to == Sleeping) {
         return;
      }
      if (direction_ == 0) {
         return;
      }
      if (steps_to_move_ == 0) {
         direction_ = 0;
         return;
      }

      /* ChronoState next_state; */

      switch (where_to) {
      case Default:
         setup_();
         /* current_pos_ -= direction_; */
         fallthrough

      case StepOverflow4:
         set_state_(Step1); // because of arithmetic state change below
         fallthrough

      case Step1:
         do_step_(0, 1, 0, 1); // May become StepUnderflow
         break;

      case Step2:
         do_step_(0, 1, 1, 0);
         break;

      case Step3:
         do_step_(1, 0, 1, 0);
         break;

      case StepUnderflow:
         set_state_(Step4); // for aritmetic state change below
         fallthrough

      case Step4:
         do_step_(1, 0, 0, 1);
         break;
      }

      #ifdef DEBUG
      default:
         fatal("LeanStepper4B: Shit up creek!");
      #endif
   }

 private:
   fn setup_() -> Void {
      dioPinMode<Pin1>(OUTPUT);
      dioPinMode<Pin2>(OUTPUT);
      dioPinMode<Pin3>(OUTPUT);
      dioPinMode<Pin4>(OUTPUT);
   }

   inl do_step_(Bool b1, Bool b2, Bool b3, Bool b4) -> Void {
      current_pos_ += direction_; // This fulfills the step to be performed
      steps_to_move_ -= direction_;
      set_bit_pattern_(b1, b2, b3, b4);
      go_after_sleep(get_state_() + direction_, pulse_width_);
   }

   inl power_down_() -> Void {
      set_bit_pattern_(0, 0, 0, 0);
   }

   inl set_bit_pattern_(Bool b1, Bool b2, Bool b3, Bool b4) -> Void {
      /* U8 bits = ((U8(b1) << 0) | (U8(b2) << 1) | (U8(b3) << 2) | (U8(b4) << 3)); */

      // if    numbers_sequential(P1,P2,P3,P4)
      //   and P1 in {22, 26, etc. pins for Mega...}
      // {
      // do-composite-port-set()   -  OVERKILL OPT ATM!!!

      // Simply switching between px and py as first arg to dwf change the hex
      // size from 7906B to 6900B (!)
      /* auto px = Pin1 + U8(0.01 * rand()); */
      /* auto py = Pin1; */
      /* digitalWriteFast<px>((bits & (1 << 0)) ? HIGH : LOW); */

      /* digitalWriteFast<Pin1>((bits & (1 << 0)) ? HIGH : LOW); */
      /* digitalWriteFast<Pin2>((bits & (1 << 1)) ? HIGH : LOW); */
      /* digitalWriteFast<Pin3>((bits & (1 << 2)) ? HIGH : LOW); */
      /* digitalWriteFast<Pin4>((bits & (1 << 3)) ? HIGH : LOW); */

      /* digitalWriteFast(Pin1, (bits & (1 << 0)) ? HIGH : LOW); */
      /* digitalWriteFast(Pin2, (bits & (1 << 1)) ? HIGH : LOW); */
      /* digitalWriteFast(Pin3, (bits & (1 << 2)) ? HIGH : LOW); */
      /* digitalWriteFast(Pin4, (bits & (1 << 3)) ? HIGH : LOW); */

      /* digitalWriteFast<Pin1, Pin2, Pin3, Pin4>(bits ? 1 : 0); */

      dioWrite<Pin1, Pin2, Pin3, Pin4>(b1, b2, b3, b4);
   }
};

#endif
