#pragma once

#include "BasicActors.h"
#include <iostream>
#include <iomanip>

namespace PhysicsEngine
{
	using namespace std;

	//a list of colours: Circus Palette
	static const PxVec3 color_palette[] = { PxVec3(46.f / 255.f,9.f / 255.f,39.f / 255.f),PxVec3(217.f / 255.f,0.f / 255.f,0.f / 255.f),
		PxVec3(255.f / 255.f,45.f / 255.f,0.f / 255.f),PxVec3(255.f / 255.f,140.f / 255.f,54.f / 255.f),PxVec3(4.f / 255.f,117.f / 255.f,111.f / 255.f) };
	

	//pyramid vertices
	static PxVec3 pyramid_verts[] = { PxVec3(0,1,0), PxVec3(1,0,0), PxVec3(-1,0,0), PxVec3(0,0,1), PxVec3(0,0,-1) };
	//pyramid triangles: a list of three vertices for each triangle e.g. the first triangle consists of vertices 1, 4 and 0
	//vertices have to be specified in a counter-clockwise order to assure the correct shading in rendering
	static PxU32 pyramid_trigs[] = { 1, 4, 0, 3, 1, 0, 2, 3, 0, 4, 2, 0, 3, 2, 1, 2, 4, 1 };

	class Pyramid : public ConvexMesh
	{
	public:
		Pyramid(PxTransform pose = PxTransform(PxIdentity), PxReal density = 1.f) :
			ConvexMesh(vector<PxVec3>(begin(pyramid_verts), end(pyramid_verts)), pose, density)
		{
		}
	};

	class PyramidStatic : public TriangleMesh
	{
	public:
		PyramidStatic(PxTransform pose = PxTransform(PxIdentity)) :
			TriangleMesh(vector<PxVec3>(begin(pyramid_verts), end(pyramid_verts)), vector<PxU32>(begin(pyramid_trigs), end(pyramid_trigs)), pose)
		{
		}
	};

	struct FilterGroup
	{
		enum Enum
		{
			ACTOR0 = (1 << 0),
			ACTOR1 = (1 << 1),
			ACTOR2 = (1 << 2)
			//add more if you need
		};
	};

	///An example class showing the use of springs (distance joints).
	class Trampoline
	{
		vector<DistanceJoint*> springs;
		
	public:

		DynamicBox *top;
		Box* bottom; 
		Trampoline(const PxTransform& pose = PxTransform(PxVec3(0.f, 10.f, 10.f)), const PxVec3& dimensions = PxVec3(5.f, 5.f, 5.f), PxReal stiffness = 1.f, PxReal damping = 1.f)
		{
			PxReal thickness = .1f;
			bottom = new Box(PxTransform(pose.p + PxVec3(0.f, thickness, 0.f), pose.q), PxVec3(dimensions.x, thickness, dimensions.z));
			top = new DynamicBox(PxTransform(pose.p + PxVec3(0.f, dimensions.y + thickness, 0.f), pose.q), PxVec3(dimensions.x, thickness, dimensions.z));
			springs.resize(4);
			springs[0] = new DistanceJoint(bottom, PxTransform(pose.p + PxVec3(dimensions.x, thickness, dimensions.z), pose.q), top, PxTransform(pose.p + PxVec3(dimensions.x, -dimensions.y, dimensions.z), pose.q));
			springs[1] = new DistanceJoint(bottom, PxTransform(pose.p + PxVec3(dimensions.x, thickness, -dimensions.z), pose.q), top, PxTransform(pose.p + PxVec3(dimensions.x, -dimensions.y, -dimensions.z), pose.q));
			springs[2] = new DistanceJoint(bottom, PxTransform(pose.p + PxVec3(-dimensions.x, thickness, dimensions.z), pose.q), top, PxTransform(pose.p + PxVec3(-dimensions.x, -dimensions.y, dimensions.z), pose.q));
			springs[3] = new DistanceJoint(bottom, PxTransform(pose.p + PxVec3(-dimensions.x, thickness, -dimensions.z), pose.q), top, PxTransform(pose.p + PxVec3(-dimensions.x, -dimensions.y, -dimensions.z), pose.q));

			for (unsigned int i = 0; i < springs.size(); i++)
			{
				springs[i]->Stiffness(stiffness);
				springs[i]->Damping(damping);
			}
		}

		void AddToScene(Scene* scene)
		{
			scene->Add(bottom);
			scene->Add(top);

		}

		~Trampoline()
		{
			for (unsigned int i = 0; i < springs.size(); i++)
				delete springs[i];
		}
	};

	///A customised collision class, implemneting various callbacks
	class MySimulationEventCallback : public PxSimulationEventCallback
	{
	public:
		//an example variable that will be checked in the main simulation loop
		bool trigger;

		MySimulationEventCallback() : trigger(false) {}

		///Method called when the contact with the trigger object is detected.
		virtual void onTrigger(PxTriggerPair* pairs, PxU32 count)
		{
			//you can read the trigger information here
			for (PxU32 i = 0; i < count; i++)
			{
				//filter out contact with the planes
				if (pairs[i].otherShape->getGeometryType() != PxGeometryType::ePLANE)
				{
					//check if eNOTIFY_TOUCH_FOUND trigger
					if (pairs[i].status & PxPairFlag::eNOTIFY_TOUCH_FOUND)
					{
						cerr << "onTrigger::eNOTIFY_TOUCH_FOUND" << endl;
						trigger = true;
					}
					//check if eNOTIFY_TOUCH_LOST trigger
					if (pairs[i].status & PxPairFlag::eNOTIFY_TOUCH_LOST)
					{
						cerr << "onTrigger::eNOTIFY_TOUCH_LOST" << endl;
						trigger = false;
					}
				}
			}
		}

		///Method called when the contact by the filter shader is detected.
		virtual void onContact(const PxContactPairHeader &pairHeader, const PxContactPair *pairs, PxU32 nbPairs)
		{
			cerr << "Contact found between " << pairHeader.actors[0]->getName() << " " << pairHeader.actors[1]->getName() << endl;

			//check all pairs
			for (PxU32 i = 0; i < nbPairs; i++)
			{
				//check eNOTIFY_TOUCH_FOUND
				if (pairs[i].events & PxPairFlag::eNOTIFY_TOUCH_FOUND)
				{
					cerr << "onContact::eNOTIFY_TOUCH_FOUND" << endl;
				}
				//check eNOTIFY_TOUCH_LOST
				if (pairs[i].events & PxPairFlag::eNOTIFY_TOUCH_LOST)
				{
					cerr << "onContact::eNOTIFY_TOUCH_LOST" << endl;
				}
			}
		}

		virtual void onConstraintBreak(PxConstraintInfo *constraints, PxU32 count) {}
		virtual void onWake(PxActor **actors, PxU32 count) {}
		virtual void onSleep(PxActor **actors, PxU32 count) {}
	};

	//A simple filter shader based on PxDefaultSimulationFilterShader - without group filtering
	static PxFilterFlags CustomFilterShader(PxFilterObjectAttributes attributes0, PxFilterData filterData0,
		PxFilterObjectAttributes attributes1, PxFilterData filterData1,
		PxPairFlags& pairFlags, const void* constantBlock, PxU32 constantBlockSize)
	{
		// let triggers through
		if (PxFilterObjectIsTrigger(attributes0) || PxFilterObjectIsTrigger(attributes1))
		{
			pairFlags = PxPairFlag::eTRIGGER_DEFAULT;
			return PxFilterFlags();
		}

		pairFlags = PxPairFlag::eCONTACT_DEFAULT;
		//enable continous collision detection
		//		pairFlags |= PxPairFlag::eCCD_LINEAR;


		//customise collision filtering here
		//e.g.

		// trigger the contact callback for pairs (A,B) where 
		// the filtermask of A contains the ID of B and vice versa.
		if ((filterData0.word0 & filterData1.word1) && (filterData1.word0 & filterData0.word1))
		{
			//trigger onContact callback for this pair of objects
			pairFlags |= PxPairFlag::eNOTIFY_TOUCH_FOUND;
			pairFlags |= PxPairFlag::eNOTIFY_TOUCH_LOST;
			//			pairFlags |= PxPairFlag::eNOTIFY_CONTACT_POINTS;
		}

		return PxFilterFlags();
	};

	///Custom scene class
	class MyScene : public Scene
	{
		Plane* plane;
		Sphere* golfBall; 
		Rectangle* rectangles; 
		RevoluteJoint* golfClub,* rotatingSpinner1,* rotatingSpinner2;
		Club* club; 
		Box* box; 
		Spinner* spinner,* spinner2; 
		Border* border; 
		MySimulationEventCallback* my_callback;
		Trampoline* trampoline;


	public:
		//specify your custom filter shader here
		//PxDefaultSimulationFilterShader by default
		MyScene() : Scene() {};
		float myForce = 0.0f;
		bool hasWon; 
		int randNum = rand() % 5; 

		///A custom scene class
		void SetVisualisation()
		{
			px_scene->setVisualizationParameter(PxVisualizationParameter::eSCALE, 1.0f);
			px_scene->setVisualizationParameter(PxVisualizationParameter::eCOLLISION_SHAPES, 1.0f);
			px_scene->setVisualizationParameter(PxVisualizationParameter::eJOINT_LOCAL_FRAMES, 1.0f);
			px_scene->setVisualizationParameter(PxVisualizationParameter::eJOINT_LIMITS, 1.0f);
		}

		//Custom scene initialisation
		virtual void CustomInit()
		{
			SetVisualisation();

			GetMaterial()->setDynamicFriction(.2f);

			///Initialise and set the customised event callback
			my_callback = new MySimulationEventCallback();
			px_scene->setSimulationEventCallback(my_callback);

			plane = new Plane();
			club = new Club(); 
			plane->Color(PxVec3(210.f / 255.f, 210.f / 255.f, 210.f / 255.f));
			Add(plane);
			


			//---------------------------------------------------------TRANSFORMS---------------------------------------------------------//
			box = new Box(PxTransform(PxVec3(.5f, .5f, 43.f)));
			golfBall = new Sphere(PxTransform(PxVec3(.5f, 5.0f, -28.0f)), 1.1f); 
			border = new Border(PxTransform(PxVec3(.5f, .5f, .5f))); 
			rectangles = new Rectangle(PxTransform(PxVec3(.5f, .5f, .5f)));
			spinner = new Spinner(PxTransform(PxVec3(.5f, .5f, .5f)));
			spinner2 = new Spinner(PxTransform(PxVec3(.5f, .5f, .5f)));
			trampoline = new Trampoline(PxTransform(PxVec3(0.f, 3.f, 15.f), PxQuat(PxPi / 2, PxVec3(1.f, 0.f, 0.f))), PxVec3(8.f, 3.f, 3.f), 410.0f, 0.3f);
			//-------------------------------------------------------------------------------------------------------------------------------//





			//---------------------------------------------------------MATERIALS---------------------------------------------------------//
			//sets a bouncy property to shapes
			PxMaterial* rectangleMaterial = CreateMaterial(0.f, .0f, 3.f); 
			PxMaterial* borderMaterial = CreateMaterial(0.f, .0f, .5f);
			PxMaterial* spinnerMaterial = CreateMaterial(0.f, .0f, .8f);
			rectangles->Material(rectangleMaterial); 
			border->Material(borderMaterial); 
			spinner->Material(spinnerMaterial);
			spinner2->Material(spinnerMaterial);
			trampoline->bottom->Material(spinnerMaterial); 
			//-------------------------------------------------------------------------------------------------------------------------------//





			//---------------------------------------------------------REVOLUTE JOINTS---------------------------------------------------------//
			//Golf club joint
			golfClub = new RevoluteJoint(NULL, PxTransform(PxVec3(0.f, 16.f, 0.f), PxQuat(PxPi / 2, PxVec3(0.f, 1.f, 0.f))), club, PxTransform(PxVec3(-25.f, 1.f, 0.f)));
			golfClub->DriveVelocity(0.0f); 

			//first rotating spinner
			rotatingSpinner1 = new RevoluteJoint(NULL, PxTransform(PxVec3(35.f, 0.1f, 50.f), PxQuat(PxPi / 2, PxVec3(0.f, 0.f, 1.f))), spinner, PxTransform(PxVec3(0.f, 5.0f, 0.f)));
			rotatingSpinner1->DriveVelocity(PxReal(7.0f));

			//second rotating spinner
			rotatingSpinner2 = new RevoluteJoint(NULL, PxTransform(PxVec3(-35.f, 0.1f, -30.f), PxQuat(PxPi / 2, PxVec3(0.f, 0.f, 1.f))), spinner2, PxTransform(PxVec3(0.f, 5.0f, 0.f)));
			rotatingSpinner2->DriveVelocity(PxReal(-3.0f));
			//-------------------------------------------------------------------------------------------------------------------------------//




			//---------------------------------------------------------COLOURS---------------------------------------------------------//
			golfBall->Color(color_palette[0]);
			box->Color(color_palette[1]);
			border->Color(color_palette[4]);
			trampoline->bottom->Color(color_palette[2]); 
			trampoline->top->Color(color_palette[3]);
			//-------------------------------------------------------------------------------------------------------------------------------//




			//Set trigger for the 'hole' 
			box->SetTrigger(1);




			//---------------------------------------------------------ADDS---------------------------------------------------------//
			//Add actors to scene 
			Add(golfBall); 
			Add(border); 
			Add(rectangles); 
			Add(box); 
			Add(spinner); 
			Add(spinner2); 
			Add(club);
			trampoline->AddToScene(this);
			//-------------------------------------------------------------------------------------------------------------------------------//
		}


		//adds force to club
		void push()
		{
			//does drive velocity == motorised joint? 
			golfClub->DriveVelocity(-myForce);
		}


		//Function to change box position between win states
		void swichBoxPosition()
		{
			switch (randNum)
			{
			case 0:
				box = new Box(PxTransform(PxVec3(.5f, .5f, 43.f)));
				break;
			case 1:
				box = new Box(PxTransform(PxVec3(.5f, .5f, 43.f)));
				break;
			case 2:
				box = new Box(PxTransform(PxVec3(.5f, .5f, 43.f)));
				break;
			case 3:
				box = new Box(PxTransform(PxVec3(.5f, .5f, 43.f)));
				break;
			case 4:
				box = new Box(PxTransform(PxVec3(.5f, .5f, 43.f)));
				break;
			case 5:
				box = new Box(PxTransform(PxVec3(.5f, .5f, 43.f)));
				break;
			default:
				break;
			}
		}

		//Custom udpate function
		virtual void CustomUpdate()
		{
			//makes sure force stays between -8 and 8
			if (myForce >= 8)
			{
				myForce = 8; 
			}

			if (myForce <= -8)
			{
				myForce = -8; 
			}

			//checks trigger
			if (my_callback->trigger)
			{
				//if golf ball has collided with hole
				hasWon = true; 
				((PxRigidDynamic*)golfBall->Get())->putToSleep(); 
			}
		}
	};
}
