using UnityEngine;
using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using System.Text;

public class XRHandMultiClientServer : MonoBehaviour
{
    public OVRCameraRig cameraRig;  // 에디터에서 연결
    public OVRCustomSkeleton leftHandSkeleton;
    public OVRCustomSkeleton rightHandSkeleton;
    public int LISTEN_PORT = 9001;
    public float timeoutSeconds = 5.0f;

    private UdpClient udpSocket;
    private IPEndPoint listenEndPoint;
    private ConcurrentDictionary<IPEndPoint, float> clientTimestamps = new();
    private Thread listenThread;
    private bool running = true;
    

    void Start()
    {
        udpSocket = new UdpClient(LISTEN_PORT);
        listenEndPoint = new IPEndPoint(IPAddress.Any, 0);

        listenThread = new Thread(ListenForClients);
        listenThread.IsBackground = true;
        listenThread.Start();

        Debug.Log($"[UDP Server] Listening on port {LISTEN_PORT}");
    }

    float[] GetHeadsetPoseArray()
    {
        float[] arr = new float[7]; // pos(3) + rot(4)

        if (cameraRig == null || cameraRig.centerEyeAnchor == null)
            return arr;
    
        Vector3 pos = cameraRig.centerEyeAnchor.position;
        Quaternion rot = cameraRig.centerEyeAnchor.rotation;

        arr[0] = pos.x;
        arr[1] = pos.y;
        arr[2] = pos.z;
        arr[3] = rot.x;
        arr[4] = rot.y;
        arr[5] = rot.z;
        arr[6] = rot.w;

        return arr;
    }

    void ListenForClients()
    {
        while (running)
        {
            try
            {
                byte[] req = udpSocket.Receive(ref listenEndPoint);
                string msg = Encoding.ASCII.GetString(req);

                if (msg.StartsWith("ping"))
                {
                    // 신규 등록 로그는 ContainsKey 체크 먼저
                    bool isNew = !clientTimestamps.ContainsKey(listenEndPoint);
                    clientTimestamps[listenEndPoint] = Time.time;

                    if (isNew)
                        Debug.Log($"[UDP Server] 신규 클라이언트 등록: {listenEndPoint}");
                }
            }
            catch (SocketException ex)
            {
                if (ex.SocketErrorCode != SocketError.Interrupted)
                    Debug.LogWarning($"[UDP Server] 수신 오류: {ex.Message}");
            }
        }
    }

    void FixedUpdate()
    {
        // ✅ FixedUpdate에서 데이터 전송 (50Hz)
        byte[] packet = PrepareHandDataPacket();
        foreach (var client in clientTimestamps.Keys)
            udpSocket.Send(packet, packet.Length, client);
    }

    void Update()
    {
        // 타임아웃 5초 이상은 제거
        List<IPEndPoint> expired = new();
        foreach (var kv in clientTimestamps)
        {
            if (Time.time - kv.Value > timeoutSeconds)
                expired.Add(kv.Key);
        }
        foreach (var ep in expired)
        {
            if (clientTimestamps.TryRemove(ep, out _))
            {
                Debug.Log($"[UDP Server] 클라이언트 타임아웃: {ep}");
            }
        }
    }

    byte[] PrepareHandDataPacket()
    {
        // 각 손 데이터 준비 (없으면 0 채움)
        float[] left = GetHandArray(leftHandSkeleton);
        float[] right = GetHandArray(rightHandSkeleton);
        float[] head = GetHeadsetPoseArray();

        byte[] header = System.Text.Encoding.ASCII.GetBytes("HND0");
        byte[] footer = System.Text.Encoding.ASCII.GetBytes("HND1");
        double timestamp = (DateTime.UtcNow - new DateTime(1970, 1, 1)).TotalSeconds;
        byte[] ts_bytes = BitConverter.GetBytes(timestamp);
        
        // 총 패킷 크기 계산: header(4) + ts(8) + 2*26*3*4 + footer(4)
        int floatCount = left.Length + right.Length +head.Length; // 182 + 182 + 7 = 371
        byte[] payload = new byte[4 + 8 + floatCount * 4 + 4]; // 1500 bytes

        int ptr = 0;
        Array.Copy(header, 0, payload, ptr, 4); ptr += 4;
        Array.Copy(ts_bytes, 0, payload, ptr, 8); ptr += 8;
        
        // 왼손 먼저(26x3)
        for (int i = 0; i < left.Length; i++)
        {
            Array.Copy(BitConverter.GetBytes(left[i]), 0, payload, ptr, 4);
            ptr += 4;
        }
        // 오른손 (26x3)
        for (int i = 0; i < right.Length; i++)
        {
            Array.Copy(BitConverter.GetBytes(right[i]), 0, payload, ptr, 4);
            ptr += 4;
        }

        // 헤드셋 pose (pos + rot)
        for (int i = 0; i < head.Length; i++)
        {
            Array.Copy(BitConverter.GetBytes(head[i]), 0, payload, ptr, 4);
            ptr += 4;
        }

        Array.Copy(footer, 0, payload, ptr, 4); ptr += 4;

        return payload;
    }


    float[] GetHandArray(OVRCustomSkeleton skel)
    {
        // 손목 pos(3) + rot(4) + 각 finger 상대 pos(3) + rot(4) × 25개
        float[] arr = new float[1 * 3 + 1 * 4 + 25 * (3 + 4)];

        if (skel == null || skel.CustomBones == null || skel.CustomBones.Count < 26)
            return arr;

        var root = skel.CustomBones[0];
        Vector3 rootPos = root.position;
        Quaternion rootRot = root.rotation;

        // 0. 손목 위치 (월드 기준)
        arr[0] = rootPos.x;
        arr[1] = rootPos.y;
        arr[2] = rootPos.z;

        // 1. 손목 회전 (쿼터니언)
        arr[3] = rootRot.x;
        arr[4] = rootRot.y;
        arr[5] = rootRot.z;
        arr[6] = rootRot.w;

        int ptr = 7;
        for (int i = 1; i < 26; i++)
        {
            var bone = skel.CustomBones[i];
            if (!bone) continue;

            // ✅ 회전 기준 상대 위치 (손목 기준 좌표계)
            Vector3 relPos = Quaternion.Inverse(rootRot) * (bone.position - rootPos);
            Quaternion relRot = Quaternion.Inverse(rootRot) * bone.rotation;

            arr[ptr++] = relPos.x;
            arr[ptr++] = relPos.y;
            arr[ptr++] = relPos.z;

            arr[ptr++] = relRot.x;
            arr[ptr++] = relRot.y;
            arr[ptr++] = relRot.z;
            arr[ptr++] = relRot.w;
        }

        return arr;
    }


    void OnApplicationQuit()
    {
        running = false;
        udpSocket?.Close();
        listenThread?.Abort(); // Unity에서는 괜찮지만 join 방식이 더 안전
    }
}
