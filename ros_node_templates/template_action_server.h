#ifndef _action_server_h_
#define _action_server_h_

// ros action server
#include <actionlib/server/simple_action_server.h>
#include <actionlib/action_definition.h>

/**
 * \brief ROS Action Server
 *
 * This class is a wrapper for the ROS Simple Action Server, thus their policies
 * are adquired. The action server logic for receiving new goals, controlling 
 * client preemption and dealing with goal status has been implemented on top of
 * the simple action server. Instead of working straight with the provided execute
 * callback, this class defines additional callbacks to the user with a more 
 * reduced scope. By using this class, the final user just needs to worry
 * about executing and cancelling the action and filling up feedback and result
 * messages. 
 *
 * This is a template class, where ActionSpec must be a class generated from an
 * action message. Goal, Result and Feedback objects are assumed to exist.
 */
template <class ActionSpec>
class ActionServer
{
  public:
   /**
    * \brief Action definitions macro
    *
    * This ROS defined macro includes convenient type definitions for Goal,
    * Result and Feedback classes to ease developement.
    */
    ACTION_DEFINITION(ActionSpec);

   /**
    * \brief Result Pointer type definition
    *
    * Additional definition for non-constant result pointers
    */
    typedef boost::shared_ptr<Result> ResultPtr;

   /**
    * \brief Result Pointer type definition
    *
    * Additional definition for non-constant feedback pointers
    */
    typedef boost::shared_ptr<Feedback> FeedbackPtr;
    
  private:
  
   /**
    * \brief Start Callback object
    *
    * Boost function object to bind user callback. The start callback is called
    * once at the beginning of the simple action server execute callback. User
    * may use the Goal object to start the action within this callback.
    *
    * The user defined function must receive a GoalConstPtr and return void.
    */
    boost::function<void (const GoalConstPtr&)> start_action_callback_;

   /**
    * \brief Stop Callback object
    *
    * Boost function object to bind user callback. The stop callback is called
    * whenever the client requests a preempt or the action fails for some other
    * reason. User may stop the action and get ready to receieve another goal.
    *
    * The user defined function takes no parameters and returns void.
    */
    boost::function<void ()> stop_action_callback_;

   /**
    * \brief Is Finished Callback object
    *
    * Boost function object to bind user callback. The is finished callback is 
    * called from the simple action server execute callback to check whether the
    * action has been finished or not. An action can finish either because it
    * accomplished its goal or because it has been aborted.
    *
    * The user defined function takes no parameters and returns a boolean
    * indicating if the action is finished or not.
    */
    boost::function<bool ()> is_finished_callback_;

   /**
    * \brief Has Succeed Callback object
    *
    * Boost function object to bind user callback. The has succeeded callback is 
    * called from the simple action server execute callback when the action has
    * finished.
    *
    * The user defined function takes no parameters and returns a boolean
    * indicating if the action successfully accomplished its goal (true) or if 
    * it was aborted (false).
    */
    boost::function<bool ()> has_succeed_callback_;
    
   /**
    * \brief Get Result Callback object
    *
    * Boost function object to bind user callback. The get result callback is 
    * called from the simple action server execute callback when the action has
    * finished. User must fill up the result object with the convenient data
    * depending on wether the action accomplished its goal or was aborted.
    *
    * The user defined function must receive a ResultPtr and return void.
    */
    boost::function<void (ResultPtr&)> get_result_callback_;
    
   /**
    * \brief Get Feedback Callback object
    *
    * Boost function object to bind user callback. The get result callback is 
    * called on each iteration of the simple action server execute callback while
    * the action is active. User must fill up the feedback object to leave it
    * ready to be published.
    *
    * The user defined function must receive a FeedbackPtr and return void.
    */
    boost::function<void (FeedbackPtr&)> get_feedback_callback_;
    
   /**
    * \brief Execute Callback
    *
    * This is the ROS simple action server execute callback which is called in a
    * separate thread whenever a new goal is received. This function implements
    * the action server logic on top of the ROS simple action server policy.
    *
    * This function controls the life-cycle of a single goal, since it is received
    * until achieves a terminal state. User callbacks are triggered to monitor
    * the goal state during the action execution.
    *
    * \param GoalConstPtr a const pointer to the received action goal
    */
    void executeCallback(const GoalConstPtr& goal);

   /**
    * \brief Action Name
    *
    * This string stores the action name used to request actions.
    */
    std::string action_name_;
    
   /**
    * \brief Simple Action Server
    *
    * The ROS simple action server is used to handle action requests. For
    * complete information on its behaviour please refer to:
    * http://www.ros.org/wiki/actionlib/DetailedDescription
    */
    actionlib::SimpleActionServer<ActionSpec> as_;

   /**
    * \brief Loop Rate
    *
    * This object controls the frequency of the execution callback loop. The
    * value can be updated by calling the member method setLoopRate.
    */
    ros::Rate loop_rate_;

   /**
    * \brief Is Started
    *
    * This variable is set to true when start() is called and to false when
    * calling shutdown().
    */
    bool is_started;

  public:
   /**
    * \brief Constructor
    *
    * This constructor mainly initializes the ROS simple action server object, 
    * as well as the action name and the execute loop rate frequency.
    *
    * \param nh a reference to the node handle object to manage all ROS topics.
    * \param action_name name of the action
    */
    ActionServer(ros::NodeHandle & nh, const std::string & action_name);

   /**
    * \brief Destructor
    *
    * This destructor is called when the object is about to be destroyed.
    *
    */
    ~ActionServer(void);
    
   /**
    * \brief Start Action
    *
    * This function checks whether all user callbacks have been registered and 
    * then starts the simple action server.
    */
    void start(void);

   /**
    * \brief Server has been started?
    *
    * This function returns true if start() has been previously called.
    */
    bool isStarted(void) const;
   
   /**
    * \brief Stop Action
    *
    * Calls the ROS simple action server to shutdown the current action
    */
    void shutdown(void);

   /**
    * \brief Abort Action
    *
    * Calls the ROS simple action server to abort current action
    *
    * \param  result An optional result to send back to any clients of the goal
    * \param  text An optional text message to send back to any clients of the goal
    */
    void setAborted(const Result& result = Result(), const std::string& text = std::string(""));

   /**
    * \brief Preempt Action
    *
    * Calls the ROS simple action server to preempt current action
    *
    * \param  result An optional result to send back to any clients of the goal
    * \param  text An optional text message to send back to any clients of the goal
    */
    void setPreempted(const Result& result = Result(), const std::string& text = std::string(""));

   /**
    * \brief Action is Active
    *
    * Allows polling implementations to query about the status of the current goal
    * @return True if a goal is active, false otherwise
    */
    bool isActive(void);

   /**
    * \brief Set Loop Rate
    *
    * This function sets the loop rate for the execute callback
    *
    */
    void setLoopRate(ros::Rate r);

   /**
    * \brief Register Start Callback
    *
    * This method let's the user register the start callback method
    *
    * \param cb user callback function with input parameter GoalConstPtr
    */
    void registerStartCallback(boost::function<void (const GoalConstPtr&)> cb);
    
   /**
    * \brief Register Stop Callback
    *
    * This method let's the user register the stop callback method
    *
    * \param cb user callback function with no input parameters
    */
    void registerStopCallback(boost::function<void ()> cb);
    
   /**
    * \brief Register Is Finished Callback
    *
    * This method let's the user register the is finished callback method
    *
    * \param cb user callback function with no input parameters, returns boolean
    */
    void registerIsFinishedCallback( boost::function<bool ()> cb);
    
   /**
    * \brief Register Has Succeed Callback
    *
    * This method let's the user register the has succeed callback method
    *
    * \param cb user callback function with no input parameters, returns boolean
    */
    void registerHasSucceedCallback( boost::function<bool ()> cb);
    
   /**
    * \brief Register Get Result Callback
    *
    * This method let's the user register the get result callback method
    *
    * \param cb user callback function with input parameter ResultPtr
    */
    void registerGetResultCallback(boost::function<void (ResultPtr&)> cb);
    
   /**
    * \brief Register Get Feedback Callback
    *
    * This method let's the user register the get feedback callback method
    *
    * \param cb user callback function with input parameter FeedbackPtr
    */
    void registerGetFeedbackCallback(boost::function<void (FeedbackPtr&)> cb);
    
};

template <class ActionSpec>
ActionServer<ActionSpec>::ActionServer(ros::NodeHandle & nh, const std::string & aname) :
  action_name_(aname),
  as_(nh, action_name_, boost::bind(&ActionServer<ActionSpec>::executeCallback, this, _1), false),
  loop_rate_(10),
  is_started(false)
{
  ROS_DEBUG("ActionServer::Constructor");
}

template <class ActionSpec>
ActionServer<ActionSpec>::~ActionServer(void)
{
//   std::cout << "ActionServer::Destructor:: active=" << as_.isActive() << std::endl;
//   if( as_.isActive() )
//     as_.setAborted();
//   as_.shutdown();
  ROS_DEBUG("ActionServer::Destructor");
}

template <class ActionSpec>
void ActionServer<ActionSpec>::setLoopRate(ros::Rate r)
{
  loop_rate_ = r;
}

template <class ActionSpec>
void ActionServer<ActionSpec>::start(void)
{
  ROS_DEBUG("ActionServer::Start");

  // check if all callbacks have been registered
  if( start_action_callback_ && 
      stop_action_callback_  &&
      is_finished_callback_  && 
      has_succeed_callback_  && 
      get_result_callback_   && 
      get_feedback_callback_ )
  {
    // launch action server
    as_.start();
    is_started = true;
  }
  else
    ROS_FATAL("Some Callbacks have not been registered yet!");
}

template <class ActionSpec>
bool ActionServer<ActionSpec>::isStarted(void) const
{
  return is_started;
}

template <class ActionSpec>
void ActionServer<ActionSpec>::shutdown(void)
{
  as_.shutdown();
  is_started = false;
}

template <class ActionSpec>
void ActionServer<ActionSpec>::setAborted(const Result& result, const std::string& text)
{
  as_.setAborted(result, text);
}

template <class ActionSpec>
void ActionServer<ActionSpec>::setPreempted(const Result& result, const std::string& text)
{
  as_.setPreempted(result, text);
}

template <class ActionSpec>
bool ActionServer<ActionSpec>::isActive(void)
{
  return as_.isActive();
}

template <class ActionSpec>
void ActionServer<ActionSpec>::executeCallback(const GoalConstPtr& goal)
{
  ROS_DEBUG("ActionServer::executeCallback::start");
  
  try
  {
    // init action
    start_action_callback_(goal);

    // boolean to control action status
    //bool is_active = true;

    // while action is active
    while(this->as_.isActive() )
    {
      ROS_DEBUG("ActionServer::executeCallback::Active!");
      
      // check if preempt has been requested
      // and if ROS connection is OK
      if( as_.isPreemptRequested() || !ros::ok() )
      {
        ROS_DEBUG("ActionServer::executeCallback::PREEMPTED!");
        //is_active = false;

        // stop action
        if(!as_.isNewGoalAvailable())
          stop_action_callback_();
        as_.setPreempted();
      }
      // check if action has finished
      else if( is_finished_callback_() )
      {
        ROS_DEBUG("ActionServer::executeCallback::FINISH!");
        //is_active = false;

        // get action result
        ResultPtr result(new Result);
        get_result_callback_(result);

        // action has been successfully accomplished
        if( has_succeed_callback_() )
        {
          ROS_DEBUG("ActionServer::executeCallback::SUCCEED!");
          // set action as succeeded
          as_.setSucceeded(*result); 
        }
        // action needs to be aborted
        else
        {
          ROS_DEBUG("ActionServer::executeCallback::ABORTED!");
          // set action as aborted
          as_.setAborted(*result); 
        }
      }
      // action has not finished yet
      else
      {
        ROS_DEBUG("ActionServer::executeCallback::feedback");
        // get action feedback and publish it
        FeedbackPtr feedback(new Feedback);
        get_feedback_callback_(feedback);
        as_.publishFeedback(*feedback); 
      }
      
      // force loop rate's frequency if convenient 
      loop_rate_.sleep();
    }
  
  }
  catch(std::exception &e)
  {
    std::cout << e.what() << std::endl;
    as_.setAborted(Result(), "something went wrong!");
  }
}

template <class ActionSpec>
void ActionServer<ActionSpec>::registerStartCallback(boost::function<void (const GoalConstPtr&)> cb)
{
  start_action_callback_ = cb;
}

template <class ActionSpec>
void ActionServer<ActionSpec>::registerStopCallback(boost::function<void ()> cb)
{
  stop_action_callback_ = cb;
}

template <class ActionSpec>
void ActionServer<ActionSpec>::registerIsFinishedCallback( boost::function<bool ()> cb)
{
  is_finished_callback_ = cb;
}

template <class ActionSpec>
void ActionServer<ActionSpec>::registerHasSucceedCallback( boost::function<bool ()> cb)
{
  has_succeed_callback_ = cb;
}

template <class ActionSpec>
void ActionServer<ActionSpec>::registerGetResultCallback(boost::function<void (ResultPtr&)> cb)
{
  get_result_callback_ = cb;
}

template <class ActionSpec>
void ActionServer<ActionSpec>::registerGetFeedbackCallback(boost::function<void (FeedbackPtr&)> cb)
{
  get_feedback_callback_ = cb;
}

#endif
