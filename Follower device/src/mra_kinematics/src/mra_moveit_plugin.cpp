#include <class_loader/class_loader.h>

//#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_kdl.h>
#include <kdl_parser/kdl_parser.hpp>
//fr
#include <kdl/frames_io.hpp>

// URDF, SRDF
#include <urdf_model/model.h>
#include <srdfdom/model.h>

#include <moveit/rdf_loader/rdf_loader.h>

// MRA kin
#include <mra_kinematics/mra_moveit_plugin.h>
#include <mra_kinematics/mra_kin.h>

#define debug

//register KDLKinematics as a KinematicsBase implementation
CLASS_LOADER_REGISTER_CLASS(mra_kinematics::MRAKinematicsPlugin, kinematics::KinematicsBase)

namespace mra_kinematics
{
    //构造函数
    MRAKinematicsPlugin::MRAKinematicsPlugin():active_(false) {}	//（false表示未准备好）

    //为总链随机获取一组关节数值
    void MRAKinematicsPlugin::getRandomConfiguration(KDL::JntArray &jnt_array, bool lock_redundancy) const
    {
        std::vector<double> jnt_array_vector(dimension_, 0.0);	//用0对整条链进行初始化
        state_->setToRandomPositions(joint_model_group_);
        state_->copyJointGroupPositions(joint_model_group_, &jnt_array_vector[0]);
        for (std::size_t i = 0; i < dimension_; ++i)
        {
            if (lock_redundancy)
                if (isRedundantJoint(i))	//如果锁定冗余并且关节为冗余的
                    continue;
            jnt_array(i) = jnt_array_vector[i];	//未出现以上两种状况，则赋值给jnt_array
        }
    }

    //判断是否为冗余关节
    bool MRAKinematicsPlugin::isRedundantJoint(unsigned int index) const
    {
        for (std::size_t j=0; j < redundant_joint_indices_.size(); ++j)
            if (redundant_joint_indices_[j] == index)
                return true;
        return false;
    }

    //获得冗余构型
    void MRAKinematicsPlugin::getRandomConfiguration(const KDL::JntArray &seed_state,
                                                     const std::vector<double> &consistency_limits,
                                                     KDL::JntArray &jnt_array,
                                                     bool lock_redundancy) const
    {
        std::vector<double> values(dimension_, 0.0);  //数值
        std::vector<double> near(dimension_, 0.0);    //
        for (std::size_t i = 0 ; i < dimension_; ++i)
            near[i] = seed_state(i);    //初始构型

        // Need to resize the consistency limits to remove mimic joints
        //重设一致性检查,移除被动关节
        std::vector<double> consistency_limits_mimic;
        for(std::size_t i = 0; i < dimension_; ++i)
        {
            if(!mimic_joints_[i].active)
                continue;
            consistency_limits_mimic.push_back(consistency_limits[i]);
        }

        joint_model_group_->getVariableRandomPositionsNearBy(state_->getRandomNumberGenerator(), values, near, consistency_limits_mimic);

        for (std::size_t i = 0; i < dimension_; ++i)
        {
            bool skip = false;
            if (lock_redundancy)
                for (std::size_t j = 0; j < redundant_joint_indices_.size(); ++j)
                    if (redundant_joint_indices_[j] == i)
                    {
                        skip = true;
                        break;
                    }
            if (skip)
                continue;
            jnt_array(i) = values[i];
        }
    }

    //检查一致性
    bool MRAKinematicsPlugin::checkConsistency(const KDL::JntArray& seed_state,
                                               const std::vector<double> &consistency_limits,
                                               const KDL::JntArray& solution) const
    {
        for (std::size_t i = 0; i < dimension_; ++i)
            if (fabs(seed_state(i) - solution(i)) > consistency_limits[i])
                return false;
        return true;
    }

    //初始化整个类
    bool MRAKinematicsPlugin::initialize(const std::string &robot_description,
                                         const std::string& group_name,
                                         const std::string& base_frame,
                                         const std::string& tip_frame,
                                         double search_discretization)
    {
        //初始化一些参数
        setValues(robot_description, group_name, base_frame, tip_frame, search_discretization);

				ros::NodeHandle private_handle("~");
				//while(1){std::cout << "good" << std::endl;}
        //加载机器人描述,robot_description: The string name corresponding to the ROS param where the URDF is loaded
        rdf_loader::RDFLoader rdf_loader(robot_description_);
        //加载srdf
        const boost::shared_ptr<srdf::Model> &srdf = rdf_loader.getSRDF();//boost
        //根据ROS版本选择使用boost库还是std库?
#if ROS_VERSION_MINIMUM(1, 13, 0)
        //加载urdf到urdf模型
        const std::shared_ptr<urdf::ModelInterface>& urdf_model = rdf_loader.getURDF();
#else
        const boost::shared_ptr<urdf::ModelInterface>& urdf_model = rdf_loader.getURDF();//boost
#endif
        //如果未成功加载模型则报错
        if (!urdf_model || !srdf)
        {
            ROS_ERROR_NAMED("kdl","URDF and SRDF must be loaded for KDL kinematics solver to work.");
            return false;
        }

        //重设机器人模型，重新指向新的模型
        robot_model_.reset(new robot_model::RobotModel(urdf_model, srdf));

				//根据moveit里定义好的组名比如arm或者hand，获得关节组的模型
        robot_model::JointModelGroup* joint_model_group = robot_model_->getJointModelGroup(group_name);
        if (!joint_model_group)
            return false;

				//确定关节组是否为链式,因为也有可能是树形结构,这个一般不用担心,除非使用moveit配置助手的时候配置错了
        if(!joint_model_group->isChain())
        {
            ROS_ERROR_NAMED("kdl","Group '%s' is not a chain", group_name.c_str());
            return false;
        }
        //确定chain包含不止一个DOF
        if(!joint_model_group->isSingleDOFJoints())
        {
            ROS_ERROR_NAMED("kdl","Group '%s' includes joints that have more than 1 DOF", group_name.c_str());
            return false;
        }

        //建立kdl树
        KDL::Tree kdl_tree;

        //从urdf模型解析出kdl树
        if (!kdl_parser::treeFromUrdfModel(*urdf_model, kdl_tree))
        {
            ROS_ERROR_NAMED("kdl","Could not initialize tree object");
            return false;
        }
        if (!kdl_tree.getChain(base_frame_, getTipFrame(), kdl_chain_))
        {
            ROS_ERROR_NAMED("kdl","Could not initialize chain object");
            return false;
				}
				//fr
				ROS_ERROR("load JOINT:%d, SEGMENT:%d", kdl_tree.getNrOfJoints(), kdl_tree.getNrOfSegments());

        //维数是在这里确定的，包括active和mimic关节（主动关节和从动关节?）
        dimension_ = joint_model_group->getActiveJointModels().size() + joint_model_group->getMimicJointModels().size();
				//fr
				ROS_ERROR("dimension = %d", dimension_);

				//从关节组模型中开始补充(获取)逆运动学求解信息
				for (std::size_t i=0; i < joint_model_group->getJointModels().size(); ++i)
        {
            //关节类型是转动关节 或 移动关节，就把这个关节信息加入 运动链逆解求解信息里
            if(joint_model_group->getJointModels()[i]->getType() == moveit::core::JointModel::REVOLUTE || joint_model_group->getJointModels()[i]->getType() == moveit::core::JointModel::PRISMATIC)
						{
								//关节名称信息
								ik_chain_info_.joint_names.push_back(joint_model_group->getJointModelNames()[i]);
								//关节限制信息
								const std::vector<moveit_msgs::JointLimits> &jvec = joint_model_group->getJointModels()[i]->getVariableBoundsMsg();
								//fr
								std::cout << "边界信息:Joint" << i+1
													<< " " << jvec.size() << " "
													<< jvec.begin()->max_position << " " << jvec.begin()->min_position
													<< std::endl;
								//jvec.begin是边界信息,end不知道是啥,这里查看了size
                ik_chain_info_.limits.insert(ik_chain_info_.limits.end(), jvec.begin(), jvec.end());
            }
        }

				//正运动学求解信息的 关节 名和限位信息可从逆运动学信息里获取
        fk_chain_info_.joint_names = ik_chain_info_.joint_names;
        fk_chain_info_.limits = ik_chain_info_.limits;

				//fr
				ROS_ERROR("getTipFrame:");
				std::cout << getTipFrame() << " " << tip_frame_ << std::endl;
				//看最后一个link(应该是Link7)是否在本组内
        if(!joint_model_group->hasLinkModel(getTipFrame()))
        {
            ROS_ERROR_NAMED("kdl","Could not find tip name in joint group '%s'", group_name.c_str());
            return false;
        }
				//把最后一个杆件的名称放入逆运动学求解信息
				ik_chain_info_.link_names.push_back(getTipFrame());
				//正运动学求解信息 杆件名称向量 和关节组模型里的名称完全一致就行
        fk_chain_info_.link_names = joint_model_group->getLinkModelNames();
				//fr
				ROS_ERROR("此时ik_chain_info_.link_names里有%d个信息", (int)ik_chain_info_.link_names.size());

				//补充关节限位信息前,先设定维度
        joint_min_.resize(ik_chain_info_.limits.size());
        joint_max_.resize(ik_chain_info_.limits.size());

				//补充关节限位信息
        for(unsigned int i=0; i < ik_chain_info_.limits.size(); i++)
        {
            joint_min_(i) = ik_chain_info_.limits[i].min_position;
            joint_max_(i) = ik_chain_info_.limits[i].max_position;
        }

        // Get Solver Parameters
        int max_solver_iterations;
        double epsilon;
        bool position_ik;

				//从参数服务器获取上面定义的这三个变量,如果这三个参数在服务器里没有定义,则用缺省值(param()函数的第三个参数)
				//fr
				//这个参数放在private空间里,看不到??
				private_handle.setParam("max_solver_iterations",300);
				private_handle.setParam("/z",300);
				bool isnt = private_handle.param("max_solver_iterations", max_solver_iterations, 500);
				//fr
				int yesOno = isnt ? 1 : 0;
				ROS_ERROR("%d存在max_solver_iterations参数,为:%d", yesOno, max_solver_iterations);
				//rf
        private_handle.param("epsilon", epsilon, 1e-5);
        private_handle.param(group_name+"/position_only_ik", position_ik, false);
        ROS_DEBUG_NAMED("kdl","Looking in private handle: %s for param name: %s",
                        private_handle.getNamespace().c_str(),
                        (group_name+"/position_only_ik").c_str());

        if(position_ik)
            ROS_INFO_NAMED("kdl","Using position only ik");

        num_possible_redundant_joints_ = kdl_chain_.getNrOfJoints() - joint_model_group->getMimicJointModels().size() - (position_ik? 3:6);

        // Check for mimic joints
        bool has_mimic_joints = joint_model_group->getMimicJointModels().size() > 0;
        std::vector<unsigned int> redundant_joints_map_index;

        std::vector<kdl_kinematics_plugin::JointMimic> mimic_joints;
        unsigned int joint_counter = 0;
        for (std::size_t i = 0; i < kdl_chain_.getNrOfSegments(); ++i)
        {
            const robot_model::JointModel *jm = robot_model_->getJointModel(kdl_chain_.segments[i].getJoint().getName());

            //first check whether it belongs to the set of active joints in the group
            if (jm->getMimic() == NULL && jm->getVariableCount() > 0)
            {
                kdl_kinematics_plugin::JointMimic mimic_joint;
                mimic_joint.reset(joint_counter);
                mimic_joint.joint_name = kdl_chain_.segments[i].getJoint().getName();
                mimic_joint.active = true;
                mimic_joints.push_back(mimic_joint);
                ++joint_counter;
                continue;
            }
            if (joint_model_group->hasJointModel(jm->getName()))
            {
                if (jm->getMimic() && joint_model_group->hasJointModel(jm->getMimic()->getName()))
                {
                    kdl_kinematics_plugin::JointMimic mimic_joint;
                    mimic_joint.reset(joint_counter);
                    mimic_joint.joint_name = kdl_chain_.segments[i].getJoint().getName();
                    mimic_joint.offset = jm->getMimicOffset();
                    mimic_joint.multiplier = jm->getMimicFactor();
                    mimic_joints.push_back(mimic_joint);
                    continue;
                }
            }
        }
        for (std::size_t i = 0; i < mimic_joints.size(); ++i)
        {
            if(!mimic_joints[i].active)
            {
                const robot_model::JointModel* joint_model = joint_model_group->getJointModel(mimic_joints[i].joint_name)->getMimic();
                for(std::size_t j=0; j < mimic_joints.size(); ++j)
                {
                    if(mimic_joints[j].joint_name == joint_model->getName())
                    {
                        mimic_joints[i].map_index = mimic_joints[j].map_index;
                    }
                }
            }
        }
        mimic_joints_ = mimic_joints;

        // Setup the joint state groups that we need
        state_.reset(new robot_state::RobotState(robot_model_));
        state_2_.reset(new robot_state::RobotState(robot_model_));

        // Store things for when the set of redundant joints may change
        position_ik_ = position_ik;
        joint_model_group_ = joint_model_group;
        max_solver_iterations_ = max_solver_iterations;
        epsilon_ = epsilon;

        private_handle.param<std::string>("arm_prefix", arm_prefix_, "");

				//核对关节名称7个
				ur_joint_names_.push_back(arm_prefix_ + "Joint1");
				ur_joint_names_.push_back(arm_prefix_ + "Joint2");
				ur_joint_names_.push_back(arm_prefix_ + "Joint3");
				ur_joint_names_.push_back(arm_prefix_ + "Joint4");
				ur_joint_names_.push_back(arm_prefix_ + "Joint5");
				ur_joint_names_.push_back(arm_prefix_ + "Joint6");
				ur_joint_names_.push_back(arm_prefix_ + "Joint7");

				//核对杆件名称8个
				ur_link_names_.push_back(arm_prefix_ + "base_link");       // 0
				ur_link_names_.push_back(arm_prefix_ + "Link1");    // 1
				ur_link_names_.push_back(arm_prefix_ + "Link2");   // 2
				ur_link_names_.push_back(arm_prefix_ + "Link3");  // 3
				ur_link_names_.push_back(arm_prefix_ + "Link4");    // 4
				ur_link_names_.push_back(arm_prefix_ + "Link5");    // 5
				ur_link_names_.push_back(arm_prefix_ + "Link6");    // 6
				ur_link_names_.push_back(arm_prefix_ + "Link7");    // 7
				//ur_link_names_.push_back(arm_prefix_ + "ee_link");         // 8

        ur_joint_inds_start_ = getJointIndex(ur_joint_names_[0]);

        // check to make sure the serial chain is properly defined in the model
        int cur_ur_joint_ind, last_ur_joint_ind = ur_joint_inds_start_;
				for(int i=1; i<7; i++) {
            cur_ur_joint_ind = getJointIndex(ur_joint_names_[i]);
            if(cur_ur_joint_ind < 0) {
                ROS_ERROR_NAMED("kdl",
                                "Kin chain provided in model doesn't contain standard MRA joint '%s'.",
                                ur_joint_names_[i].c_str());
                return false;
            }
            if(cur_ur_joint_ind != last_ur_joint_ind + 1) {
                ROS_ERROR_NAMED("kdl",
                                "Kin chain provided in model doesn't have proper serial joint order: '%s'.",
                                ur_joint_names_[i].c_str());
                return false;
            }
            last_ur_joint_ind = cur_ur_joint_ind;
        }

        // if successful, the kinematic chain includes a serial chain of the MRA joints

        kdl_tree.getChain(getBaseFrame(), ur_link_names_.front(), kdl_base_chain_);	//kdl_tree是kdl_parser根据urdf解析出来的
        kdl_tree.getChain(ur_link_names_.back(), getTipFrame(), kdl_tip_chain_);

        // weights for redundant solution selection
				//为关节增加权值，这里可以看到是7个关节
				ik_weights_.resize(7);
        if(private_handle.hasParam("ik_weights")) {
            private_handle.getParam("ik_weights", ik_weights_);
        } else {
            ik_weights_[0] = 1.0;
            ik_weights_[1] = 1.0;
            ik_weights_[2] = 0.1;
            ik_weights_[3] = 0.1;
            ik_weights_[4] = 0.3;
            ik_weights_[5] = 0.3;
        }

        active_ = true;
        ROS_DEBUG_NAMED("kdl","KDL solver initialized");
        return true;
    }

    bool MRAKinematicsPlugin::setRedundantJoints(const std::vector<unsigned int> &redundant_joints)
    {
        if(num_possible_redundant_joints_ < 0)
        {
            ROS_ERROR_NAMED("kdl","This group cannot have redundant joints");
            return false;
        }
        if(redundant_joints.size() > num_possible_redundant_joints_)
        {
            ROS_ERROR_NAMED("kdl","This group can only have %d redundant joints", num_possible_redundant_joints_);
            return false;
        }
        std::vector<unsigned int> redundant_joints_map_index;
        unsigned int counter = 0;
        for(std::size_t i=0; i < dimension_; ++i)
        {
            bool is_redundant_joint = false;
            for(std::size_t j=0; j < redundant_joints.size(); ++j)
            {
                if(i == redundant_joints[j])
                {
                    is_redundant_joint = true;
                    counter++;
                    break;
                }
            }
            if(!is_redundant_joint)
            {
                // check for mimic
                if(mimic_joints_[i].active)
                {
                    redundant_joints_map_index.push_back(counter);
                    counter++;
                }
            }
        }
        for(std::size_t i=0; i < redundant_joints_map_index.size(); ++i)
            ROS_DEBUG_NAMED("kdl","Redundant joint map index: %d %d", (int) i, (int) redundant_joints_map_index[i]);

        redundant_joints_map_index_ = redundant_joints_map_index;
        redundant_joint_indices_ = redundant_joints;
        return true;
    }

    int MRAKinematicsPlugin::getJointIndex(const std::string &name) const
    {
        for (unsigned int i=0; i < ik_chain_info_.joint_names.size(); i++) {
            if (ik_chain_info_.joint_names[i] == name)
                return i;
        }
        return -1;
    }

    int MRAKinematicsPlugin::getKDLSegmentIndex(const std::string &name) const
    {
        int i=0;
        while (i < (int)kdl_chain_.getNrOfSegments()) {
            if (kdl_chain_.getSegment(i).getName() == name) {
                return i+1;
            }
            i++;
        }
        return -1;
    }

    bool MRAKinematicsPlugin::timedOut(const ros::WallTime &start_time, double duration) const
    {
        return ((ros::WallTime::now()-start_time).toSec() >= duration);
    }
    //对应的ur_moveit_plugin.h中只提供了这一个getPositionIK方法的接口
    bool MRAKinematicsPlugin::getPositionIK(const geometry_msgs::Pose &ik_pose,
                                            const std::vector<double> &ik_seed_state,
                                            std::vector<double> &solution,
                                            moveit_msgs::MoveItErrorCodes &error_code,
                                            const kinematics::KinematicsQueryOptions &options) const
    {
        const IKCallbackFn solution_callback = 0;	//回调函数为空
        std::vector<double> consistency_limits;	//一致性限制？

        return searchPositionIK(ik_pose,
                                ik_seed_state,
                                default_timeout_,	//类的成员变量
                                solution,
                                solution_callback,	//为空
                                error_code,
                                consistency_limits,	//
                                options);
    }

    bool MRAKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                               const std::vector<double> &ik_seed_state,
                                               double timeout,
                                               std::vector<double> &solution,
                                               moveit_msgs::MoveItErrorCodes &error_code,
                                               const kinematics::KinematicsQueryOptions &options) const
    {
        const IKCallbackFn solution_callback = 0;
        std::vector<double> consistency_limits;

        return searchPositionIK(ik_pose,
                                ik_seed_state,
                                timeout,
                                solution,
                                solution_callback,	//回调函数为空
                                error_code,
                                consistency_limits,	//空
                                options);
    }

    bool MRAKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                               const std::vector<double> &ik_seed_state,
                                               double timeout,
                                               const std::vector<double> &consistency_limits,
                                               std::vector<double> &solution,
                                               moveit_msgs::MoveItErrorCodes &error_code,
                                               const kinematics::KinematicsQueryOptions &options) const
    {
        const IKCallbackFn solution_callback = 0;
        return searchPositionIK(ik_pose,
                                ik_seed_state,
                                timeout,
                                solution,
                                solution_callback,	//空
                                error_code,
                                consistency_limits,
                                options);
    }

    bool MRAKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                               const std::vector<double> &ik_seed_state,
                                               double timeout,
                                               std::vector<double> &solution,
                                               const IKCallbackFn &solution_callback,
                                               moveit_msgs::MoveItErrorCodes &error_code,
                                               const kinematics::KinematicsQueryOptions &options) const
    {
        std::vector<double> consistency_limits;
        return searchPositionIK(ik_pose,
                                ik_seed_state,
                                timeout,
                                solution,
                                solution_callback,
                                error_code,
                                consistency_limits,	//
                                options);
    }

    bool MRAKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                               const std::vector<double> &ik_seed_state,
                                               double timeout,
                                               const std::vector<double> &consistency_limits,
                                               std::vector<double> &solution,
                                               const IKCallbackFn &solution_callback,
                                               moveit_msgs::MoveItErrorCodes &error_code,
                                               const kinematics::KinematicsQueryOptions &options) const
    {
        return searchPositionIK(ik_pose,
                                ik_seed_state,
                                timeout,
                                solution,
                                solution_callback,
                                error_code,
                                consistency_limits,
                                options);
    }

		//定义了一个比较函数
    typedef std::pair<int, double> idx_double;
		//如果true,则数组里的顺序也是l在前,r在后
    bool comparator(const idx_double& l, const idx_double& r)
    { return l.second < r.second; }

    //第5个searchPositionIK，跟前一个的参数类型的顺序不同，算是重载
    bool MRAKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                               const std::vector<double> &ik_seed_state,
                                               double timeout,
                                               std::vector<double> &solution,
                                               const IKCallbackFn &solution_callback,
                                               moveit_msgs::MoveItErrorCodes &error_code,
                                               const std::vector<double> &consistency_limits,
                                               const kinematics::KinematicsQueryOptions &options) const
    {
				using std::cout;
				using std::endl;
        //记录当前时间，从当前开始计时.
        ros::WallTime n1 = ros::WallTime::now();
        //看一下求解器是否配置好.
        if(!active_) {
            ROS_ERROR_NAMED("kdl","kinematics not active");
            error_code.val = error_code.NO_IK_SOLUTION;
            return false;
        }

        //检查维数是否符合.
        if(ik_seed_state.size() != dimension_) {
            ROS_ERROR_STREAM_NAMED("kdl","Seed state must have size " << dimension_ << " instead of size " << ik_seed_state.size());
            error_code.val = error_code.NO_IK_SOLUTION;
            return false;
        }

				//如果一致性限制向量非空且该一致性限制向量的维数不对.
        if(!consistency_limits.empty() && consistency_limits.size() != dimension_) {
            ROS_ERROR_STREAM_NAMED("kdl","Consistency limits be empty or must have size " << dimension_ << " instead of size " << consistency_limits.size());
            error_code.val = error_code.NO_IK_SOLUTION;
            return false;
        }

        //根据维数，建立关节数组
        KDL::JntArray jnt_seed_state(dimension_);
				//用初始的可能位形ik_seed_state（vector类型），给关节数组赋值
        for(int i=0; i<dimension_; i++)
            jnt_seed_state(i) = ik_seed_state[i];

        //逆解数组调整大小
        solution.resize(dimension_);

				//迭代求正解,两个求解类
#ifdef debug
				ROS_WARN("kdl_base_chain_: %d 个关节", kdl_base_chain_.getNrOfJoints());
				ROS_WARN("kdl_tip_chain_: %d 个关节", kdl_tip_chain_.getNrOfJoints());
				ROS_WARN("kdl_chain_: %d 个关节", kdl_chain_.getNrOfJoints());
#endif
				KDL::ChainFkSolverPos_recursive fk_solver_base(kdl_base_chain_);	//为0
				KDL::ChainFkSolverPos_recursive fk_solver_tip(kdl_tip_chain_);	//为0

        //用的还是kdl里存储关节量的数据结构，
        KDL::JntArray jnt_pos_test(jnt_seed_state);	//初始状态
				KDL::JntArray jnt_pos_base(ur_joint_inds_start_);	//ur基座关节 ur_joint_inds_start_=0
				KDL::JntArray jnt_pos_tip(dimension_ - 7 - ur_joint_inds_start_);	//ur除了手臂关节之外和基座关节之外的其他末端关节
				//fr
				ROS_WARN("inds & dimension");
				cout << ur_joint_inds_start_<< endl << dimension_ << endl;

        //两个坐标系，基坐标系和末端坐标系
        KDL::Frame pose_base, pose_tip;

        KDL::Frame kdl_ik_pose;	//手相对于世界坐标系的位姿
        KDL::Frame kdl_ik_pose_ur_chain;	//ur的末端相对于ur基座的位姿
        double homo_ik_pose[4][4];	//齐次矩阵
				double q_ik_sols[8][7]; // maximum of 8 IK solutions，8个解最后进行挑选
        uint16_t num_sols;

				//为何一直在找
        while(1) {
            if(timedOut(n1, timeout)) {	//只要未超时则继续
                ROS_DEBUG_NAMED("kdl","IK timed out");
                error_code.val = error_code.TIMED_OUT;
                return false;
            }

            /////////////////////////////////////////////////////////////////////////////
            // find transformation from robot base to MRA base and MRA tip to robot tip
            //从总链中取出基底关节
            for(uint32_t i=0; i<jnt_pos_base.rows(); i++)
								jnt_pos_base(i) = jnt_pos_test(i);
            //从总链中取出末端附加链
            for(uint32_t i=0; i<jnt_pos_tip.rows(); i++)
								jnt_pos_tip(i) = jnt_pos_test(i + ur_joint_inds_start_ + 7);
            //先将总链赋给solution作为初始值
            for(uint32_t i=0; i<jnt_seed_state.rows(); i++)
                solution[i] = jnt_pos_test(i);

            //根据初始关节角值先计算基底位姿（初始）
						//fr
						//ROS_ERROR("pose_base:");
						//std::cout << pose_base << std::endl;
						//rf
            if(fk_solver_base.JntToCart(jnt_pos_base, pose_base) < 0) {
                ROS_ERROR_NAMED("kdl", "Could not compute FK for base chain");
                return false;
            }
						//fr
//						for(std::size_t i = 0; i < kdl_chain_.segments.size(); i++){
//								std::cout << kdl_chain_.segments[i].getName();
//						}
//						ROS_ERROR("kdl_chain: joints:%d,segments:%d", kdl_chain_.getNrOfJoints(),kdl_chain_.getNrOfSegments());
						//rf
            //根据初始关节角值计算附加末端位姿（初始）
						//fr
						ROS_ERROR("pose_tip:");
						std::cout << pose_base << std::endl;
						//rf
            if(fk_solver_tip.JntToCart(jnt_pos_tip, pose_tip) < 0) {
								ROS_ERROR_NAMED("kdl", "Could not compute FK for tip chain");
								return false;
            }
            /////////////////////////////////////////////////////////////////////////////

            /////////////////////////////////////////////////////////////////////////////
            // Convert into query for analytic solver
            //把msgs类型的坐标转换成kdl::Frame类型的
            tf::poseMsgToKDL(ik_pose, kdl_ik_pose);
            //根据总链目标姿态 得到 ur链目标姿态，这里基座和末端为什么用初始值，存疑
            kdl_ik_pose_ur_chain = pose_base.Inverse() * kdl_ik_pose * pose_tip.Inverse();

            //根据ur链目标姿态产生齐次矩阵 homo_ik_pose
            kdl_ik_pose_ur_chain.Make4x4((double*) homo_ik_pose);
#if KDL_OLD_BUG_FIX
            // in older versions of KDL, setting this flag might be necessary
            for(int i=0; i<3; i++) homo_ik_pose[i][3] *= 1000; // strange KDL fix
#endif
            /////////////////////////////////////////////////////////////////////////////

            // Do the analytic IK
            double ik_arm_seed_state[7];	//是7自由度臂的关节初始值
            for(uint32_t j=0; j<7; j++)
                ik_arm_seed_state[j] = ik_seed_state[ur_joint_inds_start_ + j];
            //计算逆解，num_sols 表示解的数量
#ifdef debug
						//fr
						ROS_WARN("开始计算逆解");
						for(int i = 0; i < 4;i++){
							for(int j=0;j<4;j++)
								cout<<homo_ik_pose[i][j]<<" ";
							cout<<endl;
						}
						cout<<"初始值:"<<endl;
						for(int i; i<7; i++)
							cout<<ik_arm_seed_state[i]<<" ";
						cout<<endl;
						//rf
#endif
            num_sols = inverse(homo_ik_pose, ik_arm_seed_state, (double*) q_ik_sols);	//计算逆解
#ifdef debug
						ROS_WARN("完成逆解");
						for(unsigned i=0;i<8;i++){
							for(int j=0;j<7;j++)
								printf("%8.3lf", q_ik_sols[i][j]);
							cout<<endl;
						}
#endif

            uint16_t num_valid_sols;	//没用到
            std::vector< std::vector<double> > q_ik_valid_sols;
            for(uint16_t i=0; i<num_sols; i++)	//i代表第i组解
            {
                bool valid = true;
                std::vector< double > valid_solution;	//有效解
								valid_solution.assign(7,0.0);			//全按0

								for(uint16_t j=0; j<7; j++)
                {
#ifdef debug
										cout<<ik_chain_info_.limits[j].max_position<<" "<<ik_chain_info_.limits[j].min_position<<endl;
#endif
                    if((q_ik_sols[i][j] <= ik_chain_info_.limits[j].max_position) && (q_ik_sols[i][j] >= ik_chain_info_.limits[j].min_position))
                    {
                        valid_solution[j] = q_ik_sols[i][j];
                        valid = true;
                        continue;
                    }
                    else if ((q_ik_sols[i][j] > ik_chain_info_.limits[j].max_position) && (q_ik_sols[i][j]-2*M_PI > ik_chain_info_.limits[j].min_position))
                    {
                        valid_solution[j] = q_ik_sols[i][j]-2*M_PI;	//M_PI是math.h里的常量
                        valid = true;
                        continue;
                    }
                    else if ((q_ik_sols[i][j] < ik_chain_info_.limits[j].min_position) && (q_ik_sols[i][j]+2*M_PI < ik_chain_info_.limits[j].max_position))
                    {
                        valid_solution[j] = q_ik_sols[i][j]+2*M_PI;
                        valid = true;
                        continue;
                    }
                    else
                    {
                        valid = false;
                        break;
                    }
                }

                if(valid)
                {
                    q_ik_valid_sols.push_back(valid_solution);
								}
#ifdef debug
								if(valid) {cout<<"valid!";
									for(int i; i<7; i++)
										cout<<valid_solution[i];
									//getPositionFK();
									KDL::ChainFkSolverPos_recursive frwd_sol(kdl_chain_);
									KDL::Frame p_out;
									KDL::JntArray jntarray_valid_solution(7);
									for(int i=0;i<7;i++)
										jntarray_valid_solution(i) = valid_solution[i];
									frwd_sol.JntToCart(jntarray_valid_solution,p_out);
									ROS_WARN("正解");
									cout<<p_out;
								}
								else cout<<"invalid";
								cout<<endl;
#endif
						}


            // use weighted absolute deviations to determine the solution closest the seed state
						//以权值作为抉择目标
						std::vector<idx_double> weighted_diffs;	//pair(关节i,权值)
            for(uint16_t i=0; i<q_ik_valid_sols.size(); i++) {
							//第i个解
                double cur_weighted_diff = 0;
								//所有关节权值相加获得当前关节的总权值
								for(uint16_t j=0; j<7; j++) {
                    // solution violates the consistency_limits, throw it out
                    double abs_diff = std::fabs(ik_seed_state[ur_joint_inds_start_+j] - q_ik_valid_sols[i][j]);
										//一致性限制向量应该是空的
                    if(!consistency_limits.empty() && abs_diff > consistency_limits[ur_joint_inds_start_+j]) {
                        cur_weighted_diff = std::numeric_limits<double>::infinity();
                        break;
                    }
                    cur_weighted_diff += ik_weights_[j] * abs_diff;
                }
                weighted_diffs.push_back(idx_double(i, cur_weighted_diff));
            }

						//根据权值进行排序
            std::sort(weighted_diffs.begin(), weighted_diffs.end(), comparator);

#if 0
            printf("start\n");
            printf("                     q %1.2f, %1.2f, %1.2f, %1.2f, %1.2f, %1.2f\n", ik_seed_state[1], ik_seed_state[2], ik_seed_state[3], ik_seed_state[4], ik_seed_state[5], ik_seed_state[6]);
            for(uint16_t i=0; i<weighted_diffs.size(); i++) {
                int cur_idx = weighted_diffs[i].first;
                printf("diff %f, i %d, q %1.2f, %1.2f, %1.2f, %1.2f, %1.2f, %1.2f\n", weighted_diffs[i].second, cur_idx, q_ik_valid_sols[cur_idx][0], q_ik_valid_sols[cur_idx][1], q_ik_valid_sols[cur_idx][2], q_ik_valid_sols[cur_idx][3], q_ik_valid_sols[cur_idx][4], q_ik_valid_sols[cur_idx][5]);
            }
            printf("end\n");
#endif

						//依次在有序
            for(uint16_t i=0; i<weighted_diffs.size(); i++) {
								//如果权值无限大,舍弃这个解
                if(weighted_diffs[i].second == std::numeric_limits<double>::infinity()) {
                    // rest are infinity, no more feasible solutions
                    break;
                }

                // copy the best solution to the output
								//顺序记录当前第i小的解索引
                int cur_idx = weighted_diffs[i].first;
								//获得该解,即为函数的输出解
                solution = q_ik_valid_sols[cur_idx];
#ifdef debug
								ROS_WARN("最终逆解为:");
								for(int i=0;i<7;i++)
									printf("%8.3lf ",solution[i]);
								cout<<endl;
#endif

                // see if this solution passes the callback function test
                if(!solution_callback.empty())
                    solution_callback(ik_pose, solution, error_code);
                else
                    error_code.val = error_code.SUCCESS;

                if(error_code.val == error_code.SUCCESS) {
#if 0
                    std::vector<std::string> fk_link_names;
                    fk_link_names.push_back(ur_link_names_.back());
                    std::vector<geometry_msgs::Pose> fk_poses;
                    getPositionFK(fk_link_names, solution, fk_poses);
                    KDL::Frame kdl_fk_pose;
                    tf::poseMsgToKDL(fk_poses[0], kdl_fk_pose);
                    printf("FK(solution) - pose \n");
                    for(int i=0; i<4; i++) {
                        for(int j=0; j<4; j++)
                            printf("%1.6f ", kdl_fk_pose(i, j)-kdl_ik_pose(i, j));
                        printf("\n");
                    }
#endif
#ifdef debug
										ROS_WARN("以下是时间");
										cout<<(ros::WallTime::now()-n1).toSec()<<endl;
#endif
                    return true;
                }
            }
            // none of the solutions were both consistent and passed the solution callback

            if(options.lock_redundant_joints) {
                ROS_DEBUG_NAMED("kdl","Will not pertubate redundant joints to find solution");
                break;
            }

						if(dimension_ == 7) {
                ROS_DEBUG_NAMED("kdl","No other joints to pertubate, cannot find solution");
                break;
            }

            // randomly pertubate other joints and try again
            if(!consistency_limits.empty()) {
                getRandomConfiguration(jnt_seed_state, consistency_limits, jnt_pos_test, false);
            } else {
                getRandomConfiguration(jnt_pos_test, false);
            }
        }

        ROS_DEBUG_NAMED("kdl","An IK that satisifes the constraints and is collision free could not be found");
        error_code.val = error_code.NO_IK_SOLUTION;
        return false;
    }

    bool MRAKinematicsPlugin::getPositionFK(const std::vector<std::string> &link_names,
                                            const std::vector<double> &joint_angles,
                                            std::vector<geometry_msgs::Pose> &poses) const
    {
        ros::WallTime n1 = ros::WallTime::now();
        if(!active_)
        {
            ROS_ERROR_NAMED("kdl","kinematics not active");
            return false;
        }
        poses.resize(link_names.size());
        if(joint_angles.size() != dimension_)
        {
            ROS_ERROR_NAMED("kdl","Joint angles vector must have size: %d",dimension_);
            return false;
        }

        KDL::Frame p_out;
				//geometry_msgs::PoseStamped pose;
				//tf::Stamped<tf::Pose> tf_pose;

        KDL::JntArray jnt_pos_in(dimension_);
        for(unsigned int i=0; i < dimension_; i++)
        {
            jnt_pos_in(i) = joint_angles[i];
        }

        //这里用的是KDL的正解法，没有用ur自己的正解法
				KDL::ChainFkSolverPos_recursive fk_solver(kdl_chain_);

        bool valid = true;
        for(unsigned int i=0; i < poses.size(); i++)
        {
            ROS_DEBUG_NAMED("kdl","End effector index: %d",getKDLSegmentIndex(link_names[i]));
            if(fk_solver.JntToCart(jnt_pos_in,p_out,getKDLSegmentIndex(link_names[i])) >=0)
            {
                //把KDL格式的位姿转化为moveit的Msg格式
                tf::poseKDLToMsg(p_out,poses[i]);
            }
            else
            {
                ROS_ERROR_NAMED("kdl","Could not compute FK for %s",link_names[i].c_str());
                valid = false;
            }
        }
        return valid;
    }

    const std::vector<std::string>& MRAKinematicsPlugin::getJointNames() const
    {
        return ik_chain_info_.joint_names;
    }

    const std::vector<std::string>& MRAKinematicsPlugin::getLinkNames() const
    {
        return ik_chain_info_.link_names;
    }

} // namespace
