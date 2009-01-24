<?xml version='1.0'?>
<Project Type="Project" LVVersion="8508002">
   <Item Name="My Computer" Type="My Computer">
      <Property Name="server.app.propertiesEnabled" Type="Bool">true</Property>
      <Property Name="server.control.propertiesEnabled" Type="Bool">true</Property>
      <Property Name="server.tcp.enabled" Type="Bool">false</Property>
      <Property Name="server.tcp.port" Type="Int">0</Property>
      <Property Name="server.tcp.serviceName" Type="Str">My Computer/VI Server</Property>
      <Property Name="server.tcp.serviceName.default" Type="Str">My Computer/VI Server</Property>
      <Property Name="server.vi.callsEnabled" Type="Bool">true</Property>
      <Property Name="server.vi.propertiesEnabled" Type="Bool">true</Property>
      <Property Name="specify.custom.address" Type="Bool">false</Property>
      <Item Name="Helper VIs" Type="Folder">
         <Item Name="Team Logo.png" Type="Document" URL="Team Logo.png"/>
         <Item Name="Decode Status Byte.vi" Type="VI" URL="Decode Status Byte.vi"/>
         <Item Name="DStoPCPacketTypeDef.ctl" Type="VI" URL="DStoPCPacketTypeDef.ctl"/>
         <Item Name="Elapsed Time.vi" Type="VI" URL="Elapsed Time.vi"/>
         <Item Name="Parse Digital Module.vi" Type="VI" URL="Parse Digital Module.vi"/>
         <Item Name="Receive DS Packet.vi" Type="VI" URL="Receive DS Packet.vi"/>
         <Item Name="Update Battery Indicator.vi" Type="VI" URL="Update Battery Indicator.vi"/>
         <Item Name="App EXE.ico" Type="Document" URL="App EXE.ico"/>
      </Item>
      <Item Name="Dashboard Main.vi" Type="VI" URL="Dashboard Main.vi"/>
      <Item Name="Dependencies" Type="Dependencies">
         <Item Name="vi.lib" Type="Folder">
            <Item Name="FixBadRect.vi" Type="VI" URL="/&lt;vilib&gt;/picture/pictutil.llb/FixBadRect.vi"/>
            <Item Name="imagedata.ctl" Type="VI" URL="/&lt;vilib&gt;/picture/picture.llb/imagedata.ctl"/>
            <Item Name="Draw Flattened Pixmap.vi" Type="VI" URL="/&lt;vilib&gt;/picture/picture.llb/Draw Flattened Pixmap.vi"/>
            <Item Name="Clear Errors.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/error.llb/Clear Errors.vi"/>
            <Item Name="Directory of Top Level VI.vi" Type="VI" URL="/&lt;vilib&gt;/picture/jpeg.llb/Directory of Top Level VI.vi"/>
            <Item Name="Check Path.vi" Type="VI" URL="/&lt;vilib&gt;/picture/jpeg.llb/Check Path.vi"/>
            <Item Name="Read JPEG File.vi" Type="VI" URL="/&lt;vilib&gt;/picture/jpeg.llb/Read JPEG File.vi"/>
            <Item Name="Bit-array To Byte-array.vi" Type="VI" URL="/&lt;vilib&gt;/picture/pictutil.llb/Bit-array To Byte-array.vi"/>
            <Item Name="Create Mask By Alpha.vi" Type="VI" URL="/&lt;vilib&gt;/picture/picture.llb/Create Mask By Alpha.vi"/>
            <Item Name="Read PNG File.vi" Type="VI" URL="/&lt;vilib&gt;/picture/png.llb/Read PNG File.vi"/>
            <Item Name="IMAQ Image.ctl" Type="VI" URL="/&lt;vilib&gt;/vision/Image Controls.llb/IMAQ Image.ctl"/>
            <Item Name="Image Type" Type="VI" URL="/&lt;vilib&gt;/vision/Image Controls.llb/Image Type"/>
            <Item Name="IMAQ Create" Type="VI" URL="/&lt;vilib&gt;/Vision/Basics.llb/IMAQ Create"/>
            <Item Name="DriverStation.lvlib" Type="Library" URL="/&lt;vilib&gt;/Robotics Library/WPI/DriverStation/DriverStation.lvlib"/>
         </Item>
         <Item Name="nivissvc.dll" Type="Document" URL="nivissvc.dll"/>
      </Item>
      <Item Name="Build Specifications" Type="Build">
         <Item Name="FRC PC Dashboard" Type="EXE">
            <Property Name="App_applicationGUID" Type="Str">{F83015BD-4316-4589-ACF4-2346824587BA}</Property>
            <Property Name="App_applicationName" Type="Str">FRC Dashboard.exe</Property>
            <Property Name="App_fileDescription" Type="Str">FRC PC Dashboard</Property>
            <Property Name="App_fileType" Type="Int">1</Property>
            <Property Name="App_fileVersion.major" Type="Int">1</Property>
            <Property Name="App_INI_aliasGUID" Type="Str">{C42F7D38-1DDF-4CAC-8CD4-C90325570FAC}</Property>
            <Property Name="App_INI_GUID" Type="Str">{120E15DA-A2E7-4A5B-B876-8B31111777E4}</Property>
            <Property Name="App_internalName" Type="Str">FRC PC Dashboard</Property>
            <Property Name="App_productName" Type="Str">FRC PC Dashboard</Property>
            <Property Name="Bld_buildSpecDescription" Type="Str">Build Dashboard Main.vi into an EXE that will respond to the driver station and display robot information on a PC.</Property>
            <Property Name="Bld_buildSpecName" Type="Str">FRC PC Dashboard</Property>
            <Property Name="Bld_defaultLanguage" Type="Str"></Property>
            <Property Name="Bld_excludePolymorphicVIs" Type="Bool">true</Property>
            <Property Name="Destination[0].destName" Type="Str">FRC Dashboard.exe</Property>
            <Property Name="Destination[0].path" Type="Path">../builds/NI_AB_PROJECTNAME/FRC PC Dashboard/internal.llb</Property>
            <Property Name="Destination[0].type" Type="Str">App</Property>
            <Property Name="Destination[1].destName" Type="Str">Support Directory</Property>
            <Property Name="Destination[1].path" Type="Path">../builds/NI_AB_PROJECTNAME/FRC PC Dashboard/data</Property>
            <Property Name="Destination[2].destName" Type="Str">Team Logo</Property>
            <Property Name="Destination[2].path" Type="Path">../builds/NI_AB_PROJECTNAME/FRC PC Dashboard</Property>
            <Property Name="DestinationCount" Type="Int">3</Property>
            <Property Name="Exe_iconItemID" Type="Ref">/My Computer/Helper VIs/App EXE.ico</Property>
            <Property Name="Source[0].itemID" Type="Str">{49E9C235-54F1-491D-A808-4AEFF33245AF}</Property>
            <Property Name="Source[0].type" Type="Str">Container</Property>
            <Property Name="Source[1].destinationIndex" Type="Int">0</Property>
            <Property Name="Source[1].itemID" Type="Ref">/My Computer/Dashboard Main.vi</Property>
            <Property Name="Source[1].sourceInclusion" Type="Str">TopLevel</Property>
            <Property Name="Source[1].type" Type="Str">VI</Property>
            <Property Name="Source[2].destinationIndex" Type="Int">2</Property>
            <Property Name="Source[2].itemID" Type="Ref">/My Computer/Helper VIs/Team Logo.png</Property>
            <Property Name="Source[2].sourceInclusion" Type="Str">Include</Property>
            <Property Name="SourceCount" Type="Int">3</Property>
         </Item>
      </Item>
   </Item>
</Project>
