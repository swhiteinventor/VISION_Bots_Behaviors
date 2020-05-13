#include "foraging_loop_functions.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <footbot_foraging.h>

/****************************************/
/****************************************/

CForagingLoopFunctions::CForagingLoopFunctions() :
   m_cForagingArenaSideX(1.1f, 1.9f),
   m_cForagingArenaSideY(-0.35f, 0.35f),
   m_pcFloor(NULL),
   m_pcRNG(NULL),
   m_unCollectedFood(0),
   m_nEnergy(0),
   m_unEnergyPerFoodItem(1),
   m_unEnergyPerWalkingRobot(1) {
}

/****************************************/
/****************************************/

void CForagingLoopFunctions::Init(TConfigurationNode& t_node) {
   try {
      TConfigurationNode& tForaging = GetNode(t_node, "foraging");
      /* Get a pointer to the floor entity */
      m_pcFloor = &GetSpace().GetFloorEntity();
      /* Get the number of food items we want to be scattered from XML */
      UInt32 unFoodItems;
      GetNodeAttribute(tForaging, "items", unFoodItems);
      /* Get the number of food items we want to be scattered from XML */
      GetNodeAttribute(tForaging, "radius", m_fFoodSquareRadius);
      m_fFoodSquareRadius *= m_fFoodSquareRadius;
      /* Create a new RNG */
      m_pcRNG = CRandom::CreateRNG("argos");
      /* Distribute uniformly the items in the environment */
      for(UInt32 i = 0; i < unFoodItems; ++i) {
      m_cFoodPos.push_back(
         CVector2(m_pcRNG->Uniform(m_cForagingArenaSideX),
                  m_pcRNG->Uniform(m_cForagingArenaSideY)));
      }
      /* Get the output file name from XML */
      GetNodeAttribute(tForaging, "output", m_strOutput);
      /* Open the file, erasing its contents */
      m_cOutput.open(m_strOutput.c_str(), std::ios_base::trunc | std::ios_base::out);
      m_cOutput << "# clock\twalking\tresting\tcollected_food\tenergy" << std::endl;
      /* Get energy gain per item collected */
      GetNodeAttribute(tForaging, "energy_per_item", m_unEnergyPerFoodItem);
      /* Get energy loss per walking robot */
      GetNodeAttribute(tForaging, "energy_per_walking_robot", m_unEnergyPerWalkingRobot);

      TConfigurationNode& tPheromones = GetNode(t_node, "pheromones");
      /* Get interior field dimensions from XML */
      GetNodeAttribute(tPheromones, "interior_width", unWidth);
      GetNodeAttribute(tPheromones, "interior_height", unHeight);
      GetNodeAttribute(tPheromones, "resolution", unResolution);
      GetNodeAttribute(tPheromones, "intensity", unIntensity);
      GetNodeAttribute(tPheromones, "dissipation", unDissipation);
      GetNodeAttribute(tPheromones, "radius", unRadius);
      GetNodeAttribute(tPheromones, "strong", unStrong);

        /* dynamic 2D array assignment of pheromone matrix*/
//        PheromoneTrail = new int*[unHeight*unResolution];
//        for (int i = 0; i < unHeight*unResolution; ++i) {
//            PheromoneTrail[i] = new int[unWidth*unResolution];
//        }
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error parsing loop functions!", ex);
   }
}

/****************************************/
/****************************************/

void CForagingLoopFunctions::Reset() {
   /* Zero the counters */
   m_unCollectedFood = 0;
   m_nEnergy = 0;
   /* Close the file */
   m_cOutput.close();
   /* Open the file, erasing its contents */
   m_cOutput.open(m_strOutput.c_str(), std::ios_base::trunc | std::ios_base::out);
   m_cOutput << "# clock\twalking\tresting\tcollected_food\tenergy" << std::endl;
   /* Distribute uniformly the items in the environment */
   for(UInt32 i = 0; i < m_cFoodPos.size(); ++i) {
      m_cFoodPos[i].Set(m_pcRNG->Uniform(m_cForagingArenaSideX),
                        m_pcRNG->Uniform(m_cForagingArenaSideY));
   }

   /* Clear the pheromone map */
   PheromoneMap.clear();

}

/****************************************/
/****************************************/

void CForagingLoopFunctions::Destroy() {
   /* Close the file */
   m_cOutput.close();
}

/****************************************/
/****************************************/

CColor CForagingLoopFunctions::GetFloorColor(const CVector2& c_position_on_plane) {
   if(c_position_on_plane.GetX() < -1.0f) {
      return CColor::GRAY50;
   }

   for(UInt32 i = 0; i < m_cFoodPos.size(); ++i) {
      if((c_position_on_plane - m_cFoodPos[i]).SquareLength() < m_fFoodSquareRadius) {
         return CColor::BLACK;
      }
   }
   /* find the coordinate position after discretizing with the resolution */
   int xLoc = std::round(c_position_on_plane.GetX()*unResolution);
   int yLoc = std::round(c_position_on_plane.GetY()*unResolution);
   /* Grab the iterator of the given location; if none, find() will return the iterator that end() returns */
   std::map<std::pair<int,int>, int>::iterator itr = PheromoneMap.find(std::pair<int,int>(xLoc,yLoc));
   /* Check if the current location has a pheromone value */
   if (itr != PheromoneMap.end()) {
      /* return color of pheromone based on intensity. If above the strong threshold, the color is just yellow. */
      if (itr->second >= unStrong) {
         return CColor::YELLOW;
      }
      else {
         UInt8 alpha = 255 * (itr->second % unStrong) / unStrong;
         /* create color with alpha based on how much pheromone is left*/
         CColor mixed = CColor(255,255,0,alpha);
         return mixed.Blend(CColor::WHITE);
      }
   }
   return CColor::WHITE;
}

/****************************************/
/****************************************/

void CForagingLoopFunctions::PreStep() {
   /* Logic to pick and drop food items */
   /*
    * If a robot is in the nest, drop the food item
    * If a robot is on a food item, pick it
    * Each robot can carry only one food item per time
    */
   UInt32 unWalkingFBs = 0;
   UInt32 unRestingFBs = 0;
   /* Check whether a robot is on a food item */
   CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");

   for(CSpace::TMapPerType::iterator it = m_cFootbots.begin();
       it != m_cFootbots.end();
       ++it) {
      /* Get handle to foot-bot entity and controller */
      CFootBotEntity& cFootBot = *any_cast<CFootBotEntity*>(it->second);
      CFootBotForaging& cController = dynamic_cast<CFootBotForaging&>(cFootBot.GetControllableEntity().GetController());
      /* Count how many foot-bots are in which state */
      if(! cController.IsResting()) ++unWalkingFBs;
      else ++unRestingFBs;
      /* Get the position of the foot-bot on the ground as a CVector2 */
      CVector2 cPos;
      cPos.Set(cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
               cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
      /* Get food data */
      CFootBotForaging::SFoodData& sFoodData = cController.GetFoodData();
      /* The foot-bot has a food item */
      if(sFoodData.HasFoodItem) {
         /* Check whether the foot-bot is in the nest */
         if(cPos.GetX() < -1.0f) {
            /* Place a new food item on the ground */
            m_cFoodPos[sFoodData.FoodItemIdx].Set(m_pcRNG->Uniform(m_cForagingArenaSideX),
                                                  m_pcRNG->Uniform(m_cForagingArenaSideY));
            /* Drop the food item */
            sFoodData.HasFoodItem = false;
            sFoodData.FoodItemIdx = 0;
            ++sFoodData.TotalFoodItems;
            /* Increase the energy and food count */
            m_nEnergy += m_unEnergyPerFoodItem;
            ++m_unCollectedFood;
            /* The floor texture must be updated */
            m_pcFloor->SetChanged();
         }
         /* put the location of the pheromone trail into the hashmap, adding to the intensity if the value is already filled */
         for (int y = -1*unRadius; y <= unRadius; y++) {
            for (int x = -1*unRadius; x <= unRadius; x++) {
               /* calculate distance from center */
               float distance = sqrt((x)^2+(y)^2);
               /* find the coordinate position after discretizing with the resolution */
               int xMat = std::round(cPos.GetX()*unResolution) + x;
               int yMat = std::round(cPos.GetY()*unResolution) + y;
               /* Grab the iterator of the current location; if none find() will return the iterator that end() returns */
               std::map<std::pair<int,int>, int>::iterator itr = PheromoneMap.find(std::pair<int,int>(xMat,yMat));
               if (itr != PheromoneMap.end()) {
                  /* Add pheromone intensity to the current location */
                  itr->second += unIntensity;// - std::round(unIntensity*distance/sqrt(2*(unRadius)^2));
               }
                  /* If the current location is devoid of pheromones */
               else {
                  /* add pheromone at the current location to the hashmap */
                  PheromoneMap.insert(std::pair<std::pair<int,int>,int>(std::pair<int,int>(xMat,yMat), unIntensity )); //- std::round(unIntensity*distance/sqrt(2*(unRadius)^2))));
               }
            }
         }
      }
      else {
         /* The foot-bot has no food item */
         /* Check whether the foot-bot is out of the nest */
         if(cPos.GetX() > -1.0f) {
            /* Check whether the foot-bot is on a food item */
            bool bDone = false;
            for(size_t i = 0; i < m_cFoodPos.size() && !bDone; ++i) {
               if((cPos - m_cFoodPos[i]).SquareLength() < m_fFoodSquareRadius) {
                  /* If so, we move that item out of sight */
                  m_cFoodPos[i].Set(100.0f, 100.f);
                  /* The foot-bot is now carrying an item */
                  sFoodData.HasFoodItem = true;
                  sFoodData.FoodItemIdx = i;
                  /* The floor texture must be updated */
                  m_pcFloor->SetChanged();
                  /* We are done */
                  bDone = true;
               }
            }
         }
      }
   }
   /* Update energy expediture due to walking robots */
   m_nEnergy -= unWalkingFBs * m_unEnergyPerWalkingRobot;
   /* Output stuff to file */
   m_cOutput << GetSpace().GetSimulationClock() << "\t"
             << unWalkingFBs << "\t"
             << unRestingFBs << "\t"
             << m_unCollectedFood << "\t"
             << m_nEnergy << std::endl;

}
/****************************************/
/****************************************/

void CForagingLoopFunctions::PostStep() {

   /* Create an iterator for looping through the pheromone map */
   std::map<std::pair<int,int>, int>::iterator it = PheromoneMap.begin();

   /* Loop through the entire pheromone map to dissipate pheromones */
   while (it != PheromoneMap.end()){
      /* Read out the pheromone map */
//      std::cout << it->first.first << ", " << it->first.second << " :: " << it->second << std::endl;
      /* Reduce pheromone by the dissipation rate */
      it->second -= unDissipation;
      /* If the pheromone reaches zero, delete the item from the map */
      if (it->second <= 0) {
         it = PheromoneMap.erase(it);
      }
      else{
         /* Increment the Iterator to point to next entry */
         it ++;
      }
   }

   /* The floor texture must be updated */
   m_pcFloor->SetChanged();

}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CForagingLoopFunctions, "foraging_loop_functions")
