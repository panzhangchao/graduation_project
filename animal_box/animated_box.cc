#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <ignition/math.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
 
namespace gazebo
{
  class AnimatedBox : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
 
        this->model = _parent;
 
        //使用PoseAnimation类实例化一个对象，然后通过三个参数可设置运动模型名称，运动持续时间以及是否循环执行
        gazebo::common::PoseAnimationPtr anim(new gazebo::common::PoseAnimation("test", 32.0, true));  
        
        //声明一个控制模型位姿的对象
        gazebo::common::PoseKeyFrame *key;  
 
 
        //设置模型到达某位姿的时刻
        key = anim->CreateKeyFrame(0);
        key->Translation(ignition::math::Vector3d(5, -4, 0));
        key->Rotation(ignition::math::Quaterniond(0, 0, 0));
 
	    key = anim->CreateKeyFrame(16.0);
        key->Translation(ignition::math::Vector3d(5, 4, 0));
        key->Rotation(ignition::math::Quaterniond(0, 0, 0));
 
        key = anim->CreateKeyFrame(32.0);
        key->Translation(ignition::math::Vector3d(5, -4, 0));
        key->Rotation(ignition::math::Quaterniond(0, 0, 0));
 
        _parent->SetAnimation(anim);
    }
 
    private: physics::ModelPtr model;
 
    //通过事件响应来更新触发程序
    private: event::ConnectionPtr updateConnection;
  };
 
  //在Gazebo仿真器中注册该插件
  GZ_REGISTER_MODEL_PLUGIN(AnimatedBox)
}
