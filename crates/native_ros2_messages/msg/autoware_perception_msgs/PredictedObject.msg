# Faithful reconstruction of autoware_perception_msgs/PredictedObject
# (AutowareFoundation/autoware_msgs). One tracked + predicted object: a UUID,
# an existence probability, a list of class hypotheses, its kinematics
# (incl. predicted paths), and its shape.

unique_identifier_msgs/UUID object_id
float32 existence_probability
autoware_perception_msgs/ObjectClassification[] classification
autoware_perception_msgs/PredictedObjectKinematics kinematics
autoware_perception_msgs/Shape shape
