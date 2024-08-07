// using Godot;
// using System.Collections.Generic;

// public partial class ChannelStatus : Panel
// {
//     private GridContainer table;
//     private List<ChannelInfo> channels;
// 	private ScrollContainer scrollContainer;

//     public override void _Ready()
//     {
// 		table = GetNode<GridContainer>("VBoxContainer/GridContainer");
//         // Create and add the header Label
//         var header = new Label();
//         header.Text = "Channel Information";
//         vbox.AddChild(header);

//         // Create and configure the GridContainer for the table
//         table = new GridContainer();
//         table.Columns = 6; // Set the number of columns
//         vbox.AddChild(table);

//         channels = new List<ChannelInfo>();

//         // Add headers to the table
//         AddHeader("Channel Name");
//         AddHeader("Message Type");
//         AddHeader("Number of Messages");
//         AddHeader("Frequency");
//         AddHeader("Bandwidth");
//         AddHeader("Dropped Packets");

//         // Example data - you can add channels dynamically as needed
//         AddChannel("Channel 1", "Type A", 100, 50.0f, 1000.0f, 0, true);
//         AddChannel("Channel 2", "Type B", 150, 75.0f, 2000.0f, 10, false);

//         // Add many channels for demonstration
//         for (int i = 3; i <= 20; i++)
//         {
//             AddChannel($"Channel {i}", $"Type {i % 5}", 100 + i, 50.0f + i, 1000.0f + i * 10, i % 3, i % 2 == 0);
//         }
//     }

//     private void AddHeader(string text)
//     {
//         var label = new Label();
//         label.Text = text;
//         label.AddColorOverride("font_color", new Color(1, 1, 1)); // White color
//         table.AddChild(label);
//     }

//     private void AddChannel(string name, string messageType, int numMessages, float frequency, float bandwidth, int droppedPackets, bool isActive)
//     {
//         var channel = new ChannelInfo(name, messageType, numMessages, frequency, bandwidth, droppedPackets, isActive);
//         channels.Add(channel);

//         var colorRect = new ColorRect();
//         colorRect.Color = isActive ? new Color(0, 1, 0) : new Color(1, 0, 0); // Green for active, Red for inactive
//         table.AddChild(colorRect);

//         var nameLabel = new Label();
//         nameLabel.Text = name;
//         table.AddChild(nameLabel);

//         var messageTypeLabel = new Label();
//         messageTypeLabel.Text = messageType;
//         table.AddChild(messageTypeLabel);

//         var numMessagesLabel = new Label();
//         numMessagesLabel.Text = numMessages.ToString();
//         table.AddChild(numMessagesLabel);

//         var frequencyLabel = new Label();
//         frequencyLabel.Text = frequency.ToString();
//         table.AddChild(frequencyLabel);

//         var bandwidthLabel = new Label();
//         bandwidthLabel.Text = bandwidth.ToString();
//         table.AddChild(bandwidthLabel);

//         var droppedPacketsLabel = new Label();
//         droppedPacketsLabel.Text = droppedPackets.ToString();
//         table.AddChild(droppedPacketsLabel);
//     }

//     private class ChannelInfo
//     {
//         public string Name;
//         public string MessageType;
//         public int NumMessages;
//         public float Frequency;
//         public float Bandwidth;
//         public int DroppedPackets;
//         public bool IsActive;

//         public ChannelInfo(string name, string messageType, int numMessages, float frequency, float bandwidth, int droppedPackets, bool isActive)
//         {
//             Name = name;
//             MessageType = messageType;
//             NumMessages = numMessages;
//             Frequency = frequency;
//             Bandwidth = bandwidth;
//             DroppedPackets = droppedPackets;
//             IsActive = isActive;
//         }
//     }
// }
