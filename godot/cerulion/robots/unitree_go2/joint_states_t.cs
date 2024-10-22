/* LCM type definition class file
 * This file was automatically generated by lcm-gen
 * DO NOT MODIFY BY HAND!!!!
 */

using System;
using System.Collections.Generic;
using System.IO;
using LCM.LCM;
 
namespace unitree_lcm
{
    public sealed class joint_states_t : LCM.LCM.LCMEncodable
    {
        public long tick;
        public double[] q_front_left;
        public double[] q_front_right;
        public double[] q_rear_left;
        public double[] q_rear_right;
 
        public joint_states_t()
        {
            q_front_left = new double[3];
            q_front_right = new double[3];
            q_rear_left = new double[3];
            q_rear_right = new double[3];
        }
 
        public static readonly ulong LCM_FINGERPRINT;
        public static readonly ulong LCM_FINGERPRINT_BASE = 0x1d53d664211e0282L;
 
        static joint_states_t()
        {
            LCM_FINGERPRINT = _hashRecursive(new List<String>());
        }
 
        public static ulong _hashRecursive(List<String> classes)
        {
            if (classes.Contains("unitree_lcm.joint_states_t"))
                return 0L;
 
            classes.Add("unitree_lcm.joint_states_t");
            ulong hash = LCM_FINGERPRINT_BASE
                ;
            classes.RemoveAt(classes.Count - 1);
            return (hash<<1) + ((hash>>63)&1);
        }
 
        public void Encode(LCMDataOutputStream outs)
        {
            outs.Write((long) LCM_FINGERPRINT);
            _encodeRecursive(outs);
        }
 
        public void _encodeRecursive(LCMDataOutputStream outs)
        {
            outs.Write(this.tick); 
 
            for (int a = 0; a < 3; a++) {
                outs.Write(this.q_front_left[a]); 
            }
 
            for (int a = 0; a < 3; a++) {
                outs.Write(this.q_front_right[a]); 
            }
 
            for (int a = 0; a < 3; a++) {
                outs.Write(this.q_rear_left[a]); 
            }
 
            for (int a = 0; a < 3; a++) {
                outs.Write(this.q_rear_right[a]); 
            }
 
        }
 
        public joint_states_t(byte[] data) : this(new LCMDataInputStream(data))
        {
        }
 
        public joint_states_t(LCMDataInputStream ins)
        {
            if ((ulong) ins.ReadInt64() != LCM_FINGERPRINT)
                throw new System.IO.IOException("LCM Decode error: bad fingerprint");
 
            _decodeRecursive(ins);
        }
 
        public static unitree_lcm.joint_states_t _decodeRecursiveFactory(LCMDataInputStream ins)
        {
            unitree_lcm.joint_states_t o = new unitree_lcm.joint_states_t();
            o._decodeRecursive(ins);
            return o;
        }
 
        public void _decodeRecursive(LCMDataInputStream ins)
        {
            this.tick = ins.ReadInt64();
 
            this.q_front_left = new double[(int) 3];
            for (int a = 0; a < 3; a++) {
                this.q_front_left[a] = ins.ReadDouble();
            }
 
            this.q_front_right = new double[(int) 3];
            for (int a = 0; a < 3; a++) {
                this.q_front_right[a] = ins.ReadDouble();
            }
 
            this.q_rear_left = new double[(int) 3];
            for (int a = 0; a < 3; a++) {
                this.q_rear_left[a] = ins.ReadDouble();
            }
 
            this.q_rear_right = new double[(int) 3];
            for (int a = 0; a < 3; a++) {
                this.q_rear_right[a] = ins.ReadDouble();
            }
 
        }
 
        public unitree_lcm.joint_states_t Copy()
        {
            unitree_lcm.joint_states_t outobj = new unitree_lcm.joint_states_t();
            outobj.tick = this.tick;
 
            outobj.q_front_left = new double[(int) 3];
            for (int a = 0; a < 3; a++) {
                outobj.q_front_left[a] = this.q_front_left[a];
            }
 
            outobj.q_front_right = new double[(int) 3];
            for (int a = 0; a < 3; a++) {
                outobj.q_front_right[a] = this.q_front_right[a];
            }
 
            outobj.q_rear_left = new double[(int) 3];
            for (int a = 0; a < 3; a++) {
                outobj.q_rear_left[a] = this.q_rear_left[a];
            }
 
            outobj.q_rear_right = new double[(int) 3];
            for (int a = 0; a < 3; a++) {
                outobj.q_rear_right[a] = this.q_rear_right[a];
            }
 
            return outobj;
        }
    }
}

