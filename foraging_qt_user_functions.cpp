#include "foraging_qt_user_functions.h"
#include <footbot_foraging.h>
#include <argos3/core/simulator/entity/controllable_entity.h>

using namespace argos;

/****************************************/
/****************************************/

CForagingQTUserFunctions::CForagingQTUserFunctions() {
   RegisterUserFunction<CForagingQTUserFunctions,CFootBotEntity>(&CForagingQTUserFunctions::Draw);
}

/****************************************/
/****************************************/

void CForagingQTUserFunctions::Draw(CFootBotEntity& c_entity) {
   CFootBotForaging& cController = dynamic_cast<CFootBotForaging&>(c_entity.GetControllableEntity().GetController());
   CFootBotForaging::SFoodData& sFoodData = cController.GetFoodData();
   if(sFoodData.HasFoodItem) {
      DrawCylinder(
         CVector3(0.0f, 0.0f, 0.3f), 
         CQuaternion(),
         0.1f,
         0.05f,
         CColor::BLUE);
   }
   /* The position of the text is expressed wrt the reference point of the footbot
       * For a foot-bot, the reference point is the center of its base.
       * See also the description in
       * $ argos3 -q foot-bot
       */
   DrawText(CVector3(0.0, 0.0, 0.3),   // position
            c_entity.GetId().c_str()); // text
}

/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(CForagingQTUserFunctions, "foraging_qt_user_functions")
