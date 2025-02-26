# Copyright (C) 2023-2024 Yucheng Liu. GNU AGPL 3.0 license.
# GNU AGPL 3.0 License: https://www.gnu.org/licenses/agpl-3.0.txt .
#
# Receives UDP packets.

Write-Output "begin UDP_Packets_Receive"


# Configure.
$Local_IP = [System.Net.IPAddress]::Parse("0.0.0.0")
$Local_Port = 50000
$Remote_IP = [System.Net.IPAddress]::Any
$Remote_Port = 0
$Local_Client = New-Object System.Net.Sockets.UdpClient
$Local_EndPoint = new-object System.Net.IPEndPoint ($Local_IP, $Local_Port)
$Local_Client.Client.Bind($Local_EndPoint)

Write-Output @"
Local_IP: $Local_IP; Local_Port: $Local_Port
Remote_IP: $Remote_IP; Remote_Port: $Remote_Port
Local_Client: $Local_Client
"@


# Connect.
# $Local_Client.Connect($Local_IP, $Local_Port)
$Local_Client.Client.ReceiveTimeout = 0
$Local_Client.Client.Blocking = $true


# Receive.
while($true) {
    $Remote_Endpoint = New-Object System.Net.IPEndPoint($Remote_IP, $Remote_Port)
    $Packet_Bytes = $Local_Client.Receive([ref]$Remote_Endpoint)
    $Packet_Texts = [System.Text.Encoding]::UTF8.GetString($Packet_Bytes)
    $Sender_IP = $Remote_Endpoint.Address.ToString()
    $Sender_Port = $Remote_Endpoint.Port.ToString()
    
    Write-Output @"
Received UDP packet
Sender_IP: $Sender_IP; Sender_Port: $Sender_Port
---- begin Packet_Texts ----
$Packet_Texts
---- end Packet_Texts ----
"@

}


# Close.
$Local_Client.Close()
Write-Output "end UDP_Packets_Receive"
