namespace fcl
{
  class Container : public CollisionGeometry
  {
  private:
    std::vector< std::shared_ptr<CollisionObject> > content;
  public:
    Container()
      {
      }

  Container(std::vector< std::shared_ptr<CollisionObject> > _content) : content(_content)
    {
    }

    virtual OBJECT_TYPE getObjectType() const { return OT_CONTAINER; }

    virtual NODE_TYPE getNodeType() const { return CONTAINER; }

    void computeLocalAABB()
    {
      for(auto collision_object_ptr : content) {
        collision_object_ptr->computeAABB();
        aabb_local += collision_object_ptr->getAABB();
      }
      aabb_center = aabb_local.center();
      // A better bound could be obtained by taking the suprema of the distance from center to any of
      // contained objects' AABB
      aabb_radius = (aabb_local.min_ - aabb_center).length();
    }

    virtual Vec3f computeCOM() const {
      Vec3f com;
      FCL_REAL volume;
      for(auto collision_object_ptr : content) {
        const std::shared_ptr<const CollisionGeometry> cgeom = collision_object_ptr->collisionGeometry();
        FCL_REAL geomvolume = cgeom->computeVolume();
        com += geomvolume * cgeom->computeCOM();
        volume += geomvolume;
      }
      if( volume < 1 - std::numeric_limits<FCL_REAL>::epsilon() ||
          volume > 1 + std::numeric_limits<FCL_REAL>::epsilon() ) {
        com *= 1/volume;
      }
      return com;
    }

    // TODO
    virtual Matrix3f computeMomentofInertia() const {
      return Matrix3f();
    }

    virtual FCL_REAL computeVolume() const {
      FCL_REAL volume;
      for(auto collision_object_ptr : content) {
        volume += collision_object_ptr->collisionGeometry()->computeVolume();
      }
      return volume;
    }

    const std::vector< std::shared_ptr<CollisionObject> >& getContent() const {
      return content;
    }
  };
} // fcl
