# Copyright (C) 2023-2024 Yucheng Liu. GNU AGPL 3.0 license.
# GNU AGPL 3.0 License: https://www.gnu.org/licenses/agpl-3.0.txt .
#
# Sends UDP packets.

Write-Output "end UDP_Packets_Send"


# Configure.
$Local_IP = [System.Net.IPAddress]::Parse("127.0.0.1")
$Local_Port = 50010
$Remote_IP = [System.Net.IPAddress]::Parse("192.168.31.240")
$Remote_Port = 50010
# $Local_EndPoint = New-Object System.Net.IPEndPoint($Local_IP, $Local_Port)
$Local_Client = New-Object System.Net.Sockets.UdpClient($Local_Port)

Write-Output @"
Local_IP: $Local_IP; Local_Port: $Local_Port
Remote_IP: $Remote_IP; Remote_Port: $Remote_Port
Local_Client: $Local_Client
"@


# Connect.
$Local_Client.Connect($Remote_IP, $Remote_Port)
$Local_Client.Client.ReceiveTimeout = 1000
$Local_Client.Client.Blocking = $false


# Send.
$Packet_Index = 0

while($true) {
    $Packet_Texts = "Packet $Packet_Index Sent`n"
    $Packet_Bytes = [System.Text.Encoding]::ASCII.GetBytes($Packet_Texts)
    # $Local_Client.GetStream().Write($Packet_Bytes, 0, $Packet_Bytes.Length)
    $Local_Client.Send($Packet_Bytes, $Packet_Bytes.Length)
    
    Write-Output @"
Sent packet
Local_IP: $Local_IP; Local_Port: $Local_Port
Remote_IP: $Remote_IP; Remote_Port: $Remote_Port
---- Begin Packet_Texts ----
$Packet_Texts
---- End Packet_Texts ----
"@

    $Packet_Index += 1
    Start-Sleep 1
}


# Close.
$Local_Client.Close()
Write-Output "end UDP_Packets_Send"
